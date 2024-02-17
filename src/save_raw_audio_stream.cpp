#include <ros/ros.h>
#include <audio_common_msgs/AudioData.h>
#include <sndfile.h>

// WAV file format
const int FORMAT_FLOAT_32 = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
const int FORMAT_INT_16 = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

class AudioWriter {
public:

    /**
     * @brief Construct a new Audio Writer object
     * 
     * @param Nh 
     * @param OutputDirectory 
     * @param PostFix 
     * @param TopicName 
     * @param QueueSize 
     */
    AudioWriter(
        ros::NodeHandle Nh,
        std::string& OutputDirectory,
        std::string& PostFix,
        std::string& TopicName,
        int& QueueSize):
        AudioWriter(Nh, OutputDirectory, PostFix, TopicName, QueueSize, false) {

    }

    /**
     * @brief Construct a new Audio Writer object
     * 
     * @param Nh 
     * @param OutputDirectory 
     * @param PostFix 
     * @param TopicName 
     * @param QueueSize 
     * @param Downsample 
     */
    AudioWriter(
        ros::NodeHandle Nh,
        std::string& OutputDirectory,
        std::string& PostFix,
        std::string& TopicName,
        int& QueueSize,
        bool Downsample):
        nh_(Nh),
        file_timestamp_("new"),
        post_fix_(PostFix),
        output_directory_(OutputDirectory),
        topic_name_(TopicName),
        queue_size_(QueueSize),
        downsample_(Downsample)
    {
        // init ROS subscriber
        sub_ = nh_.subscribe(topic_name_, queue_size_, &AudioWriter::WriteToFileDownsample, this);



        // open WAV file for writing
        sfinfo_.channels = channels_;
        sfinfo_.samplerate = out_sample_rate_;
        sfinfo_.format = out_format_;

        // create output file
        audio_file_ = output_directory_ + "/" + post_fix_ + ".wav";
        ROS_INFO("outfile: '%s'", audio_file_.c_str());
        outfile = sf_open(audio_file_.c_str(), SFM_WRITE, &sfinfo_);


        ROS_INFO("Opened file for writing");
        if (!outfile) {
            ROS_ERROR("Failed to open WAV file for writing.");
            ros::shutdown();
        }
    }

    /**
     * @brief Destroy the Audio Writer object
     * 
     */
    ~AudioWriter() {
        if (outfile) {
            sf_close(outfile);
        }
    }

    /**
     * @brief 
     * 
     * @param msg 
     */
    void WriteToFileRaw(const audio_common_msgs::AudioData::ConstPtr& msg) {
        if (outfile) {
            sf_write_raw(outfile, &msg->data[0], msg->data.size());
        }
    }

    /**
     * @brief 
     * 
     * @param msg 
     */
    void WriteToFileDownsample(const audio_common_msgs::AudioData::ConstPtr& msg) {

        // assuming the incoming audio is 32-bit float PCM with a 96000 sample rate
        const int src_sample_rate = 96000;
        const int target_sample_rate = sfinfo_.samplerate;
        int ratio = src_sample_rate / target_sample_rate; // Basic ratio for downsampling; assumes divisibility

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
        if (sf_write_short(outfile, target_data.data(), target_data.size()) != target_data.size()) {
            ROS_ERROR("Failed to write audio data to file.");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    SNDFILE* outfile;
    std::string file_timestamp_;
    std::string output_directory_;
    std::string post_fix_;
    SF_INFO sfinfo_;
    std::string audio_file_;
    std::string topic_name_;
    int queue_size_;
    bool downsample_;

    int channels_;
    int in_sample_rate_;
    int out_sample_rate_;
    int in_format_;
    int out_format_;



    void SetFormat(const std::string& Encoding, int& Format)
    {
        if(Encoding == "float"){
            Format = FORMAT_FLOAT_32;
        } 
        else if (Encoding == "pcm_16"){
            Format = FORMAT_INT_16;
        }
        else {
            ROS_ERROR("audio_encoding format error. Options: ['float', 'pcm_16']");
        }
    }

    bool LoadParam(const std::string& Param, int& ClassVariable)
    {
        // get input sample rate
        if (nh_.getParam(Param, ClassVariable)){
            return true;
        } 
        else {
            ROS_ERROR("error loading '%s'", Param.c_str());
            return false;
        }
    }

    bool SetAudioParams()
    {
        // init
        std::string in_encoding {""};
        std::string out_encoding {""};

        // get input audio encoding
        if (nh_.getParam("/audio_writer/input/audio_encoding", in_encoding)){
            SetFormat(in_encoding, in_format_);
        } else {
            ROS_ERROR("error loading '/audio_writer/input/audio_encoding'");
            return false;
        }

        // get ouput audio encoding
        if (nh_.getParam("/audio_writer/output/audio_encoding", in_encoding)){
            SetFormat(in_encoding, out_format_);
        } else {
            ROS_ERROR("error loading '/audio_writer/output/audio_encoding'");
            return false;
        }

        if (
            !LoadParam("/audio_writer/input/sample_rate", in_sample_rate_) ||
            !LoadParam("/audio_writer/output/sample_rate", out_sample_rate_) ||
            !LoadParam("/audio_writer/input/channels", channels_)) {
            return false;

        }

        return true;
    }
};

int main(int argc, char **argv) {

    // init
    ros::init(argc, argv, "save_raw_audio_stream");
    ros::NodeHandle nh;

    // load file output directory and postfix params
    std::string output_directory, post_fix, topic_name;
    int queue_size;
    bool downsample;
    if (
        nh.getParam("output_directory_", output_directory) && 
        nh.getParam("postfix_", post_fix) && 
        nh.getParam("topic_name_", topic_name) &&
        nh.getParam("queue_size_", queue_size) &&
        nh.getParam("downsample_", downsample)) {
        ROS_INFO("\n\tSubscribing to: '%s'\n\tSaving audio files to: '%s'\n\tFiles with be post fixed with: '%s'", 
        topic_name.c_str(), 
        output_directory.c_str(),
        post_fix.c_str()); 
    } 
    else {
        ROS_ERROR("Faild to load 'output_directory' and 'post_fixed' params.");
    }

    // build object
    AudioWriter audio_writer(nh, output_directory, post_fix, topic_name, queue_size, downsample);

    // ros std
    ros::spin();
    return 0;
}