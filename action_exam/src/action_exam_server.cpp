#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_exam/PrimenumberAction.h>

class PrimenumberAction
{
protected:
    ros::NodeHandle nh_; //Node handle declaration
    actionlib::SimpleActionServer<action_exam::PrimenumberAction> as_;
    std::string action_name_;  //Use as action name
    // Create messages that are used to published feedback/result
    action_exam::PrimenumberFeedback feedback_;
    action_exam::PrimenumberResult result_;

public:
    // Initialize action server (Node handle, action name, action callback function)
    PrimenumberAction(std::string name) :
        as_(nh_, name, boost::bind(&PrimenumberAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();  // Start the action server
    }

    // Callback function for when a goal is received
    void executeCB(const action_exam::PrimenumberGoalConstPtr &goal)
    {
        ros::Rate r(1); // Set a rate of 1Hz
        bool success = true;
        
        // Initialize feedback and result
        // Setting Primenumber sequence initialization
        // add first (0) and second message (1) of feedback
        feedback_.cur_num = 0;
        result_.sequence.clear();
        
        // Notify the user of the action name, goal, initial two values of Primenumber sequence
        ROS_INFO("%s: Finding prime numbers up to %d", action_name_.c_str(), goal->target);
        
        // Action contents
        for (int i = 2; i <= goal->target; i++)
        {
            // Check if preemption has been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                // Notify action cancellation
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // Action cancellation and consider action as failure and save to variable
                as_.setPreempted();
                success = false;
                break;
            }

            // Check if the number i is prime
            // Store the sum of current Primenumber number and the previous number in the feedback
            // while there is no action cancellation or the action target value is reached.
            bool isPrime = true;
            for (int j = 2; j <= sqrt(i); j++)  // Only check up to sqrt(i)
            {
                if (i % j == 0)
                {
                    isPrime = false;
                    break;
                }
            }

            if (isPrime)
            {
                ROS_INFO("Found prime number: %d", i);
                result_.sequence.push_back(i);  // Add to result
                feedback_.cur_num = i;  // Update feedback with the latest prime number
                as_.publishFeedback(feedback_);  // Publish feedback
            }

            r.sleep();  // Wait for 1 second
        }

        if (success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);  // Set the result and notify that the action succeeded
        }
    }
};

int main(int argc, char **argv)  // Node main function
{
    ros::init(argc, argv, "action_exam_server");  // Initialize the ROS node name
    PrimenumberAction prime_number_action("action_exam_action");  // Create the action server object
    ros::spin();  // Keep the node running and processing callbacks
    return 0;
}

