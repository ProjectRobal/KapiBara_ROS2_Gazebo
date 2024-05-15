
extern "C"
{
    #include <fftw3.h>
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "kapibara_interfaces/srv/emotions.hpp"
#include "kapibara_interfaces/msg/microphone.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "simd_vector.hpp"

#include "network.hpp"

#include "crossovers/onepoint.hpp"
#include "crossovers/fastuniform.hpp"
#include "crossovers/fastonepoint.hpp"
 
#include "initializers/gauss.hpp"
#include "initializers/normalized_gauss.hpp"

#include "mutatiom/gauss_mutation.hpp"

#include "activation/sigmoid.hpp"
#include "activation/relu.hpp"
#include "activation/relu_sliper.hpp"

#include "neurons/forwardneuron.hpp"

#include "layer.hpp"

#include "serializaers/network_serialize.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

/*
 It will listen for sensor data wich will be input for neural network:
    TOFs ( 4 numbers )
    Orientation ( Quanterion 4 numbers )
    Odometry ( position in 3D space 3 numbers )
    Microphone ( we will calculate FFT from data, https://www.fftw.org/ , 1024 numbers )
    Audio Coefficient ( wich channel has higher intensity 2 numbers )
    Past outputs ( 2 numbers )

    Network step will be triggered by orientation callback.

 Network output:
    linear speed ( 1 number )
    angular speed ( 1 number )

    in Twist message it will be linear(linear,0.f,0.f) and angular (0.f,0.f,angular)
    geometry_msgs/msg/Twist
 
 Reward from service Emotions.

*/

class KapiBaraMind : public rclcpp::Node
{
  
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> tof_topics;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_topic;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_topic;
  rclcpp::Subscription<kapibara_interfaces::msg::Microphone>::SharedPtr microphone_topic;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_output;

  rclcpp::Client<kapibara_interfaces::srv::Emotions>::SharedPtr emotions_provider;

  snn::SIMDVector inputs;

  snn::Network network;

  double max_linear_speed;
  double max_angular_speed;


  rclcpp::CallbackGroup::SharedPtr sub_group;
  rclcpp::CallbackGroup::SharedPtr client_group;

  void create_network()
  {
    std::shared_ptr<snn::NormalizedGaussInit> norm_gauss=std::make_shared<snn::NormalizedGaussInit>(0.f,0.01f);
    std::shared_ptr<snn::GaussInit> gauss=std::make_shared<snn::GaussInit>(0.f,0.25f);

    // intitialize network here load pre saved models etc
    auto mutation=std::make_shared<snn::GaussMutation>(0.f,0.001f,0.1f);
    auto cross=std::make_shared<snn::OnePoint>();

    auto relu=std::make_shared<snn::ReLu>();
    auto sigmoid=std::make_shared<snn::Sigmoid>();
    auto slipy=std::make_shared<snn::SlipyReLu>();

    std::shared_ptr<snn::Layer<snn::ForwardNeuron<526>,64>> first = std::make_shared<snn::Layer<snn::ForwardNeuron<526>,64>>(128,gauss,cross,mutation);
    std::shared_ptr<snn::Layer<snn::ForwardNeuron<128>,64>> layer1 = std::make_shared<snn::Layer<snn::ForwardNeuron<128>,64>>(512,gauss,cross,mutation);
    std::shared_ptr<snn::Layer<snn::ForwardNeuron<512>,64>> layer2 = std::make_shared<snn::Layer<snn::ForwardNeuron<512>,64>>(128,gauss,cross,mutation);
    std::shared_ptr<snn::Layer<snn::ForwardNeuron<128>,64>> layer3 = std::make_shared<snn::Layer<snn::ForwardNeuron<128>,64>>(64,gauss,cross,mutation);
    std::shared_ptr<snn::Layer<snn::ForwardNeuron<64>,64>> layer4 = std::make_shared<snn::Layer<snn::ForwardNeuron<64>,64>>(2,gauss,cross,mutation);

    first->setActivationFunction(slipy);
    layer1->setActivationFunction(slipy);
    layer2->setActivationFunction(slipy);
    layer3->setActivationFunction(slipy);
    layer4->setActivationFunction(relu);

    this->network.addLayer(first);
    this->network.addLayer(layer1);
    this->network.addLayer(layer2);
    this->network.addLayer(layer3);
    this->network.addLayer(layer4);
  }

  public:
  
    KapiBaraMind()
    : Node("KapiBaraMind")
    {
        this->sub_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->client_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options;
        options.callback_group = this->sub_group;

        std::vector< std::string > tofs = {"/laser_front"};
        // holds topics for tof Range data
        this->declare_parameter("tof_topics", tofs);
        this->declare_parameter("odometry_topic","/motors/odom");
        this->declare_parameter("imu_topic","/Gazebo/imu");
        this->declare_parameter("microphone_topic","/mic");

        // output settings
        this->declare_parameter("max_linear_speed",5.f);
        this->declare_parameter("max_angular_speed",4.f);

        this->max_linear_speed = this->get_parameter("max_linear_speed").as_double();
        this->max_angular_speed = this->get_parameter("max_angular_speed").as_double();


        std::string odom_topic = this->get_parameter("odometry_topic").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string mic_topic = this->get_parameter("microphone_topic").as_string();
        tofs = this->get_parameter("tof_topics").as_string_array();


        this->twist_output = this->create_publisher<geometry_msgs::msg::Twist>("/motors/cmd_vel_unstamped", 10);
        RCLCPP_INFO(this->get_logger(), "Creating publisher for outpur");

        this->odometry_topic =  this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&KapiBaraMind::odometry_callback, this, _1),options);
        RCLCPP_INFO(this->get_logger(), "Creating subscription for odometry at %s",odom_topic.c_str());

        this->orientation_topic =  this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10, std::bind(&KapiBaraMind::orientation_callback, this, _1),options);
        RCLCPP_INFO(this->get_logger(), "Creating subscription for orientation at %s",imu_topic.c_str());

        this->microphone_topic =  this->create_subscription<kapibara_interfaces::msg::Microphone>(
        mic_topic, 10, std::bind(&KapiBaraMind::microphone_callback, this, _1),options);
        RCLCPP_INFO(this->get_logger(), "Creating subscription for microphone at %s",mic_topic.c_str());

        uint8_t id=0;
        for(const std::string& tof : tofs)
        {
            this->tof_topics.push_back(this->create_subscription<sensor_msgs::msg::Range>(
        tof, 10, [this,id](const sensor_msgs::msg::Range::SharedPtr range)->void{
            this->tof_callback(range,id);
        },options));

            id++;

            RCLCPP_INFO(this->get_logger(), "Creating subscription for tof at %s",tof.c_str());
        }

        this->emotions_provider = this->create_client<kapibara_interfaces::srv::Emotions>("emotions",rmw_qos_profile_services_default,this->client_group);
        RCLCPP_INFO(this->get_logger(), "Creating service client for emotion retrival");

        this->inputs=snn::SIMDVector(0.f,526);

        this->create_network();
    }


    double get_reward()
    {
        while (!this->emotions_provider->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<kapibara_interfaces::srv::Emotions::Request>();

        auto result = this->emotions_provider->async_send_request(request);

        // Wait for the result.
        std::future_status status = result.wait_for(60s);
        if (status == std::future_status::ready)
        {
            auto emotions = result.get();

            // return emotions
            return ( -2.f*emotions->angry ) + ( -5.f*emotions->fear ) + ( 10.f*emotions->happiness ) + ( -2.f*emotions->uncertainty ) - emotions->boredom;

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get emotions");
            rclcpp::shutdown();
            return 0;
        }

    }

    void tof_callback(const sensor_msgs::msg::Range::SharedPtr range,uint8_t id)
    {
        inputs.set(range->range,id);
        RCLCPP_DEBUG(this->get_logger(), "Getting tof range from sensor: %u %f",id,range->range);
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
    {
        // I have to think about it:
        inputs.set(odometry->pose.pose.position.x/1000.f,5);
        inputs.set(odometry->pose.pose.position.y/1000.f,6);
        inputs.set(odometry->pose.pose.position.z/1000.f,7);

        RCLCPP_DEBUG(this->get_logger(), "Getting odometry");
        //RCLCPP_INFO(this->get_logger(),"x: %f y: %f z: %f",odometry->pose.pose.position.x,odometry->pose.pose.position.y,odometry->pose.pose.position.z);

    }

    number frame_activ(const number& num)
    {
        if((num>0.25f)&&(num<4.f))
        {
            return ((num-0.25f)/(4.f-0.25f) - 0.5f)*2.f;
        }
        
        return 0;
    }

    void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr orientation)
    {
        if((!std::isnan(orientation->orientation.x))&&(!std::isnan(orientation->orientation.y))&&(!std::isnan(orientation->orientation.z))&&(!std::isnan(orientation->orientation.w)))
        {
            inputs.set(orientation->orientation.x,8);
            inputs.set(orientation->orientation.y,9);
            inputs.set(orientation->orientation.z,10);
            inputs.set(orientation->orientation.w,11);
        }
        // this gives NaN values
        RCLCPP_DEBUG(this->get_logger(), "Getting orientation");
        //RCLCPP_INFO(this->get_logger(),"x: %f y: %f z: %f w: %f",orientation->orientation.x,orientation->orientation.y,orientation->orientation.z,orientation->orientation.w);


        const double reward = this->get_reward();

        RCLCPP_INFO(this->get_logger(), "Reward: %f",reward);

        this->network.applyReward(reward);

        snn::SIMDVector output = this->network.fire(this->inputs);

        geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();

        RCLCPP_INFO(this->get_logger(), "Output: 1: %Lf 2: %Lf",output[0],output[1]);

        output.set(frame_activ(output[0])*this->max_linear_speed,0);
        output.set(frame_activ(output[1])*this->max_angular_speed,1);

        /*if(output[0]>0)
        {
            output.set(output[0]-this->max_linear_speed,0);
        }

        if(output[1]>0)
        {
            output.set(output[1]-this->max_angular_speed,1);
        }

        if(abs(output[0])>this->max_linear_speed)
        {
            output.set(this->max_linear_speed*(output[0]/abs(output[0])),0);
        }

        if(abs(output[1])>this->max_angular_speed)
        {
            output.set(this->max_angular_speed*(output[1]/abs(output[1])),1);
        }*/

        twist.linear.x=output[0];
        twist.angular.z=output[1];

        RCLCPP_INFO(this->get_logger(), "Output: l: %f a: %f",twist.linear.x,twist.angular.z);

        inputs.set(output[0],524);
        inputs.set(output[1],525);

        this->twist_output->publish(twist);
    }

    // to test
    void microphone_callback(const kapibara_interfaces::msg::Microphone::SharedPtr microphone)
    {
        // do FFT transform
        uint32_t N = microphone->buffor_size;

        std::vector<int32_t> channel_left = microphone->channel1;
        std::vector<int32_t> channel_right = microphone->channel2;

        double input[N];

        double left_intenisty=0.f;
        double right_intensity=0.f;

        for(size_t i=0;i<N;++i)
        {
            double left=static_cast<double>(channel_left[i])/(2.f*INT32_MAX);
            double right=static_cast<double>(channel_right[i])/(2.f*INT32_MAX);

            input[i]=(left/2.f)+(right/2.f);

            left_intenisty+=(left*left);
            right_intensity+=(right*right);
        }

        left_intenisty=left_intenisty/static_cast<double>(N);
        right_intensity=right_intensity/static_cast<double>(N);

        fftw_plan p; 

        double out[N / 2 + 1];

        p = fftw_plan_r2r_1d(N, input, out,FFTW_DHT, FFTW_ESTIMATE);

        /*
        * Execute the dft as indicated by the plan
        */
        fftw_execute(p);

        fftw_destroy_plan(p);

        // pass outputs to inputs

        size_t pivot=(N/2 + 1);

        for(size_t i=0;i<pivot;++i)
        {
            inputs.set(out[i],12+i);
        }

        inputs.set(left_intenisty/(right_intensity+left_intenisty),12+pivot);
        inputs.set(left_intenisty/(right_intensity+left_intenisty),12+pivot+1);

    }

    ~KapiBaraMind()
    {
        snn::NetworkSerializer::save(this->network,"./checkpoint");
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<KapiBaraMind>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);

  //rclcpp::spin(std::make_shared<KapiBaraMind>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}