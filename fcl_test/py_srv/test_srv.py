import sys
import rospy
from FCL_TEST.srv import state_service



def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
 
 #因为我们已经将服务的类型声明为AddTwoInts，所以它会为您生成AddTwoIntsRequest对象（可以自由传递）
    return AddTwoIntsResponse(req.a + req.b)    # AddTwoIntsResponse由服务生成的返回函数

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')  # 声明节点为add_two_ints_server
   
    #定义服务器节点名称，服务类型，处理函数
	#处理函数调用实例化的AddTwoIntsRequest接收请求和返回实例化的AddTwoIntsResponse
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()   # 就像订阅者示例一样，rospy.spin()使代码不会退出，直到服务关闭；

if __name__ == "__main__":
    add_two_ints_server()
