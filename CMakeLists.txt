cmake_minimum_required(VERSION 3.0.2)
project(benthos_release)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BenthosReleaseCommand.msg
  BenthosReleaseStatus.msg
  # Add other message files if any
)

generate_messages(
  DEPENDENCIES
  std_msgs
  # Add other dependencies if any
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
  # Add other dependencies if needed
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add other necessary configurations, installations, etc.

