#include <ros/ros.h>
#include <audio_common_msgs/AudioData.h>
#include <sndfile.h>

// WAV file parameters
//const int FORMAT = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
const int FORMAT = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
const int CHANNELS = 1; // Adjust based on your audio data
//const int SAMPLE_RATE = 96000; // Adjust based on your audio data
const int SAMPLE_RATE = 16000; // Adjust based on your audio data

class AudioToWavConverter {
public:
    AudioToWavConverter(
        ros::NodeHandle Nh,
        std::string& OutputDirectory,
        std::string& PostFix):
        nh_(Nh),
        file_timestamp_("new"),
        post_fix_(PostFix),
        output_directory_(OutputDirectory)
    {
        // Initialize ROS subscriber
        sub_ = nh_.subscribe("/hololens/microphone/data", 1000, &AudioToWavConverter::WriteToFileDownsample, this);

        // Open WAV file for writing
        sfinfo_.channels = CHANNELS;
        sfinfo_.samplerate = SAMPLE_RATE;
        sfinfo_.format = FORMAT;
        audio_file_ = output_directory_ + "/" + post_fix_ + ".wav";
        outfile = sf_open(audio_file_.c_str(), SFM_WRITE, &sfinfo_);
        ROS_INFO("Opened file for writing");
        if (!outfile) {
            ROS_ERROR("Failed to open WAV file for writing.");
            ros::shutdown();
        }
    }

    ~AudioToWavConverter() {
        if (outfile) {
            sf_close(outfile);
        }
    }

    void WriteToFileRaw(const audio_common_msgs::AudioData::ConstPtr& msg) {
        ROS_INFO("Message heard.");
        if (outfile) {
            sf_write_raw(outfile, &msg->data[0], msg->data.size());
        }
    }

    void WriteToFileDownsample(const audio_common_msgs::AudioData::ConstPtr& msg) {

        // Assuming the incoming audio is 32-bit float PCM with a 96000 sample rate
        const int src_sample_rate = 96000;
        const int target_sample_rate = sfinfo_.samplerate;
        int ratio = src_sample_rate / target_sample_rate; // Basic ratio for downsampling; assumes divisibility

        // Convert and downsample the incoming audio data
        std::vector<float> src_data(msg->data.size() / sizeof(float));
        memcpy(src_data.data(), msg->data.data(), msg->data.size());

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
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "save_raw_audio_stream");
    ros::NodeHandle nh;

    // load file output directory and postfix
    std::string output_directory, post_fix;
    if(nh.getParam("output_directory", output_directory) && nh.getParam("postfix", post_fix)) {
        ROS_INFO("\n\tSaving audio files to: '%s'\n\tFiles with be post fixed with: '%s'", output_directory.c_str(), post_fix.c_str()); 
    } else {
        ROS_ERROR("Faild to load 'output_directory' and 'post_fixed' params.");
    }

    AudioToWavConverter converter(nh, output_directory, post_fix);
    ros::spin();
    return 0;
}