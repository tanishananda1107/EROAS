from launch import LaunchDescription
import launch_testing.actions


def generate_test_description():

    return LaunchDescription([
        launch_testing.actions.ReadyToTest()
    ]), {}
