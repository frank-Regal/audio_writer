#include <ros/ros.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/Empty.h>
#include <sndfile.h>
#include <chrono>

// WAV file format
const int FORMAT_FLOAT_32 = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
const int FORMAT_INT_16 = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

class AudioWriter {
public:

    /**
     * @brief Construct a new Audio Writer object
     * 
     * @param Nh              - ros node handle
     * @param OutputDirectory - directory where to save the audio files
     * @param PostFix         - string to post append to output file name
     * @param TopicName       - ROS topic that the audio data is being published on
     * @param QueueSize       - the queue size for the subscriber listening to the audio data
     */
    AudioWriter(ros::NodeHandle Nh, std::string& OutputDirectory, std::string& PostFix, std::string& TopicName, int& QueueSize):
        nh_(Nh),
        file_timestamp_("new"),
        post_fix_(PostFix),
        output_directory_(OutputDirectory),
        topic_name_(TopicName),
        queue_size_(QueueSize),
        downsample_(false),
        is_open_for_recording_(false)
    {
        // load all params associated with encoding and formating for audio file
        // see params/params.yaml
        if(!LoadAudioParams()){ROS_ERROR("Params not configured correctly.");}

        // determine if ros params require downsampling
        if(downsample_){
            CreateSubscriber<audio_common_msgs::AudioData>(topic_name_, queue_size_, &AudioWriter::WriteToFileDownsample, this);
            ROS_WARN("'save_raw_audio_stream' node configured for downsampling audio input.");
        }
        else {
            CreateSubscriber<audio_common_msgs::AudioData>(topic_name_, queue_size_, &AudioWriter::WriteToFileRaw, this);
            ROS_WARN("'save_raw_audio_stream' node configured to write to audio file with the same input format");
        }
    }

    /**
     * @brief Destroy the Audio Writer object
     * 
     */
    ~AudioWriter() {
        CloseFile();
    }

    /**
     * @brief Configure node to handle multiple batches of audio messages
     * 
     * Configure the node to have to std_msgs/Empty ROS msgs publish to start
     * and stop the node when in batch mode.
     * Info: in batch mode you can publish multiple batches of audio messages
     * and the node with record to multiple audio files.
     * 
     * @param StartTopic - std_msgs/Empty topic
     * @param StopTopic  - std_msgs/Empty topic
     */
    void StartMultiBatchWriter(const std::string& StartTopic, const std::string StopTopic)
    {
        CreateSubscriber<std_msgs::Empty>(StartTopic, 0, &AudioWriter::StartRecordingAudio, this);
        CreateSubscriber<std_msgs::Empty>(StopTopic, 0, &AudioWriter::StopRecordingAudio, this);
    }

    /**
     * @brief Configure node to only one batch of audio messages
     * 
     */
    void StartSingleBatchWriter()
    {
        OpenFile();
    }

private:

    // init class variables
    std::string file_timestamp_, output_directory_, post_fix_, topic_name_, filename_;
    int queue_size_, channels_, in_sample_rate_, out_sample_rate_, format_;
    bool downsample_, is_open_for_recording_;

    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subscribers_;
    SNDFILE* audio_file_;
    SF_INFO audio_file_info_;

    /**
     * @brief Write ROS message to a audio file
     * 
     * This writes to the audio file with the same format and encoding as the input
     * 
     * @param msg 
     */
    void WriteToFileRaw(const audio_common_msgs::AudioData::ConstPtr& msg) 
    {
        // o
        if (audio_file_) {
            sf_write_raw(audio_file_, &msg->data[0], msg->data.size());
        }
    }

    /**
     * @brief 
     * 
     * @param msg 
     */
    void WriteToFileDownsample(const audio_common_msgs::AudioData::ConstPtr& msg) {

        // assuming the incoming audio is 32-bit float PCM with a 96000 sample rate
        int ratio = in_sample_rate_ / out_sample_rate_; // Basic ratio for downsampling; assumes divisibility

        // convert and downsample the incoming audio data
        std::vector<float> src_data(msg->data.size() / sizeof(float));
        memcpy(src_data.data(), msg->data.data(), msg->data.size());

        // fill new array
        std::vector<short> target_data(src_data.size() / ratio);
        for (size_t i = 0; i < target_data.size(); ++i) {

            // naive downsampling - just picking every nth sample
            // TODO: consider averaging or using a resampling library like `libsamplerate`
            float sample = src_data[i * ratio];

            // Scale then normalize float values to signed 16-bit PCM encoding
            // note: maximum value of a 16-bit integer is 32767
            target_data[i] = static_cast<short>(sample * 32767.0f);
        }

        // Write downsampled data to file
        if (sf_write_short(audio_file_, target_data.data(), target_data.size()) != target_data.size()) {
            ROS_ERROR("Failed to write audio data to file. File closed.");
        }
    }

    /**
     * @brief Overloaded template function to create a subscriber
     * 
     * This is called when there is just one topic name associated with
     * the param server.
     * 
     * @tparam T           - message topic type (i.e. std_msgs/Empty)
     * @param TopicParam   - parameter server param that holds the topic name. this could be a list or a single variable
     * @param QueueSize    - message queue size
     * @param CallbackFunc - funtion you want to call when a message is received
     */
    template<typename T>
    void CreateSubscriber(
        const std::string& TopicName,
        const int& QueueSize, 
        void(AudioWriter::*CallbackFunc)(const typename T::ConstPtr&),
        AudioWriter *Obj)
    {
        ros::Subscriber sub = nh_.subscribe<T>(TopicName, QueueSize, CallbackFunc, Obj);
        subscribers_.push_back(sub);
    }

    /**
     * @brief ROS callabck to open file for recording
     * 
     * when a std_msgs/Empty message is published, create and open an audio file.
     * 
     * @param Msg - std_msgs/Empty
     */
    void StartRecordingAudio(const std_msgs::Empty::ConstPtr& Msg)
    {
        OpenFile();
    }

    /**
     * @brief ROS callback to close file for recording
     * 
     * when a std_msgs/Empty message is published, close the audio file for writing
     * 
     * @param Msg - std_msgs/Empty
     */
    void StopRecordingAudio(const std_msgs::Empty::ConstPtr& Msg)
    {
        CloseFile();
    }


    /**
     * @brief Create and open audio file
     * 
     * 1) set audio file settings
     * 2) build the file name and define output directory
     * 3) open the file for writing. Shutdown node if the file is not open.
     * 
     */
    void OpenFile()
    {
        // make sure the last file is closed.
        if(is_open_for_recording_){CloseFile();}

        // set audio output file settings
        audio_file_info_.channels = channels_;
        audio_file_info_.samplerate = out_sample_rate_;
        audio_file_info_.format = format_;

        //  build file name
        std::string timestamp, filepath;
        GetTimeStamp(timestamp);
        filename_ = timestamp + post_fix_ + ".wav";
        filepath = output_directory_ + "/" + filename_;

        // open
        audio_file_ = sf_open(filepath.c_str(), SFM_WRITE, &audio_file_info_);
        
        // check
        if (!audio_file_) {
            ROS_ERROR("Failed to open '%s' file for writing.", filename_.c_str());
            ros::shutdown();
            return;
        }

        // log
        is_open_for_recording_ = true;
        ROS_INFO("'%s' opened.\n\nWriting ...\n", filename_.c_str());
    }

    /**
     * @brief Close the audio file once finished.
     * 
     */
    void CloseFile()
    {
        if (audio_file_) {
            sf_close(audio_file_);
            is_open_for_recording_ = false;
            ROS_INFO("'%s' closed", filename_.c_str());
        }
    }

    /**
     * @brief Set the Audio Params object
     * 
     * set these parameters in 'params/params.yaml' file
     * 
     * @return true 
     * @return false 
     */
    bool LoadAudioParams()
    {
        // init
        std::string encoding {""};

        // load audio encoding params (defined in 'params/params.yaml')
        if (!LoadParam<std::string>("/audio_writer/input/audio_encoding", encoding) ||
            !LoadParam<int>("/audio_writer/input/sample_rate", in_sample_rate_) ||
            !LoadParam<int>("/audio_writer/output/sample_rate", out_sample_rate_) ||
            !LoadParam<int>("/audio_writer/input/channels", channels_)) {
            return false;
        }

        // determine if downsampling is required.
        if (in_sample_rate_ > out_sample_rate_) {
            downsample_ = true;
            SetFormat("pcm_16", format_);
        } 
        else {
            SetFormat(encoding, format_);
        }

        return true;
    }

    /**
     * @brief Template function to load multiple params at once.
     * 
     * @tparam T             - paramater type
     * @param Param          - paramater you would like to load
     * @param ClassVariable  - variable to assign parameter to
     * @return true          - if param was successfully loaded
     * @return false         - if param could not be loaded
     */
    template<typename T>
    bool LoadParam(const std::string& Param, T& ClassVariable)
    {
        if (nh_.getParam(Param, ClassVariable)) {
            return true;
        } 
        else {
            ROS_ERROR("Error loading '%s' param. Check 'params/param.yaml' file.", Param.c_str());
            return false;
        }
    }

    /**
     * @brief Set the Format object
     * 
     * @param Encoding 
     * @param Format 
     */
    void SetFormat(const std::string& Encoding, int& Format)
    {
        // set the proper formating used for SndFile library.
        // these are defined at top. See SndFile library for more details.
        if(Encoding == "float") {
            Format = FORMAT_FLOAT_32;
        } 
        else if (Encoding == "pcm_16") {
            Format = FORMAT_INT_16;
        }
        else {
            ROS_ERROR("audio_encoding format error. Current options: ['float', 'pcm_16']");
        }
    }

    /**
     * @brief Get current timestamp
     * 
     * @param TimeStamp - outputs the current ROS time
     */
    void GetTimeStamp(std::string& TimeStamp)
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S_");
        TimeStamp = ss.str();
    }
};


int main(int argc, char **argv) {

    // init
    ros::init(argc, argv, "save_raw_audio_stream");
    ros::NodeHandle nh;

    // load params required to init audio writer class
    std::string output_directory, post_fix, topic_name;
    int queue_size;
    if (nh.getParam("output_directory_", output_directory) && 
        nh.getParam("postfix_", post_fix) && 
        nh.getParam("topic_name_", topic_name) &&
        nh.getParam("queue_size_", queue_size)) {

        // if all params loaded
        ROS_INFO(
        "'save_raw_audio_stream' node configurations:\n\tSubscribing to: '%s'\n\tSaving audio files to: '%s'\n\tFiles will be post fixed with: '%s'", 
        topic_name.c_str(), 
        output_directory.c_str(),
        post_fix.c_str()); 
    } 
    else {

        // stop node if params were not loaded
        ROS_ERROR("Faild to load 'output_directory' and 'post_fixed' params. Shutting down node ...");
        ros::shutdown();
        return 1;
    }

    // create audio writer class object.
    AudioWriter audio_writer(nh, output_directory, post_fix, topic_name, queue_size);

    // determine the node setup. If start and stop topics are configured on the param server,
    // then set up the node to start and stop recording audio messages when the empty msgs are published.
    std::string start_recording_topic, stop_recording_topic;
    if (nh.getParam("start_recording_topic_", start_recording_topic) &&
        nh.getParam("stop_recording_topic_", stop_recording_topic)) {

        // if topics are configured on param server start multi batch writer
        audio_writer.StartMultiBatchWriter(start_recording_topic, stop_recording_topic);
        ROS_WARN(
            "'save_raw_audio_stream' configured for multi-batch writing. Listening to start recording topic ['%s'] and end recording topic ['%s']",
            start_recording_topic.c_str(),
            stop_recording_topic.c_str());
    } 
    else {
        ROS_WARN("'save_raw_audio_stream' configured for single-batch writing.");
        // if no start and stop topics are configured on the param server, setup single batch writer.
        audio_writer.StartSingleBatchWriter();
    }
    
    // ros std
    ros::spin();
    return 0;
}