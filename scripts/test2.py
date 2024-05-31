#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseStamped

def movebase_client():
    # ROS 노드 초기화
    rospy.init_node('movebase_client_py')

    # MoveBaseAction 서버에 연결
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # 이동 목표 설정
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # 이동 목표의 좌표계를 설정 (예: 맵 좌표계)
    goal.target_pose.header.stamp = rospy.Time.now()  # 현재 시간으로 설정

    # 이동할 위치와 방향 설정
    goal.target_pose.pose.position.x = 0.0  # X 좌표 설정
    goal.target_pose.pose.position.y = 8.0  # Y 좌표 설정
    goal.target_pose.pose.orientation.w = 1.0  # 방향 설정 (여기서는 쿼터니언 표현으로 사용하여 회전 없음을 의미)

    # 로봇에게 목표 전송
    client.send_goal(goal)

    # 로봇이 목표 지점에 도달할 때까지 대기
    client.wait_for_result()

    # 이동 작업 결과 확인
    return client.get_state() == actionlib.GoalStatus.SUCCEEDED

if __name__ == '__main__':
    try:
        # 이동 명령 실행
        result = movebase_client()
        if result:
            rospy.loginfo("목표 지점에 도달했습니다.")
        else:
            rospy.logerr("목표 지점으로의 이동에 실패했습니다.")
    except rospy.ROSInterruptException:
        rospy.logerr("프로그램이 종료되었습니다.")