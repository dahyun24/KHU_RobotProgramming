#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <action_exam/PrimenumberAction.h>
#include <cstdlib>

// Global variable to track feedback count
int get_num = 0;

// Done Callback: Called once when the action completes
void doneCb(const actionlib::SimpleClientGoalState& state, const action_exam::PrimenumberResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    for (size_t i = 0; i < result->sequence.size(); i++) {
        ROS_INFO("%lu: %d", i + 1, result->sequence[i]);
    }
}

// Active Callback: Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active.");
}

// Feedback Callback: Called every time feedback is received for the goal
void feedbackCb(const action_exam::PrimenumberFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback: current prime number is %d", feedback->cur_num);
    get_num++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_exam_client");  // Initialize ROS node

    // Action Client Declaration (Action Name: ros tutorial action)
    actionlib::SimpleActionClient<action_exam::PrimenumberAction> ac("action_exam_action", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();  //wait for the action server to start, will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // Declare the goal
    action_exam::PrimenumberGoal goal;

    if (argc != 2)
        goal.target = 20;  // Set Action Goal (Process the Primenumber sequence 20 times)
    else
        goal.target = (int)atoi(argv[1]);  // Use the command-line argument as the target

    // Send the goal to the action server and bind the callback functions
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    // Set a time limit for the action (30 seconds)
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    // Process when action results are received within the time limit for achieving the action goal
    if (finished_before_timeout)
    {
	// Receive action target status value and display on screen
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the timeout.");
    }

    return 0;
}

