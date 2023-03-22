import rospy
import sys
from FCL_TEST.srv import state_service

rospy.init_node('service_client')  # 节点初始化
print("wait for service.")
rospy.wait_for_service('word_count')  # 等待服务端声明这个服务
print("Service has started")
# 声明服务的本地代理，需要指定服务的名称（‘word_count’）和类型（WordCount）
# 这允许我们像使用本地函数一样使用服务。
word_counter = rospy.ServiceProxy('word_count', state_service)
words =''.join(sys.argv[1:])
word_count = word_counter(words)
print words, '->', word_count.count 
rospy.spin()