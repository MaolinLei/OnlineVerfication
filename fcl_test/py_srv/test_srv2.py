#!/usr/bin/env python3

import rospy
from FCL_TEST.srv import state_service,state_serviceResponse

def count_words(request):
    # 回调函数只接受WordCountRequest 类型的的参数，并返回一个WordCountResponse类型的值
    return state_serviceResponse(len(request.words.split()))

def server():
    rospy.init_node('IK_service')  # 节点初始化
    # 声明服务，服务名称 'word_count',srv类型 WordCount，回调函数 count_words.
    service = rospy.Service('IK', state_service, count_words) 
    print("Ready to service.")
    rospy.spin()

if __name__ == "__main__":
    server()   
