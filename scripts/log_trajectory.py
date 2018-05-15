#!/usr/bin/python
import argparse
import rospy
import tf
import tf.msg

class LogTrajectory:

    def __init__(self,args):
        self.args = args
        

        print('[trajectory]:  ' + args.filename)
        print('[fixed_frame]: ' + args.fixed_frame)
        print('[moving_frame]:' + args.moving_frame)
        print('[rate]:        ' + str(args.rate))

        #init ros
        rospy.init_node('log_trajectory', anonymous=True) #make node 
        self.tf_listener = tf.TransformListener()
        self.old_time = 0
      
        self.file = open(self.args.filename, 'w')                    
                           
        r = rospy.Rate(args.rate)         
        while not rospy.is_shutdown():            
            self.updateTf()    
            r.sleep()
            
        rospy.spin()    



    def updateTf(self):
        
        time_now = rospy.Time.now()        
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(self.args.fixed_frame, 
                                              self.args.moving_frame, 
                                              now, rospy.Duration(0.8))
            
            (trans,rot) = self.tf_listener.lookupTransform(self.args.fixed_frame, self.args.moving_frame, now)
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
            
            p_string = "%f, %f, %f, %f, %f, %f, %f \n" % (now.to_sec(), trans[0], trans[1], trans[2], roll, pitch, yaw)
            time_diff = now.to_sec() - self.old_time 
            self.old_time = now.to_sec()
            if (time_diff > 0):                
                self.file.write(p_string)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
            print "Error: Could not get transform from %s to %s at time %f"%(self.args.moving_frame, self.args.fixed_frame, time_now.to_sec())
            return

if __name__ == "__main__":
 
    parser = argparse.ArgumentParser(description='Logs the trajectory of an object by recording tf poses')
    parser.add_argument('--filename',     type=str,   default="logged_trajectory.csv", help='Name of the file to be logged')
    parser.add_argument('--fixed_frame',  type=str,   default="/odom",                 help='A fixed frame e.g. /odom')
    parser.add_argument('--moving_frame', type=str,   default="/base_link",            help='A moving frame e.g. /base_link')
    parser.add_argument('--rate',         type=float, default="100.0",                  help='The rate to capture the trajectory with.')

    args, unkown = parser.parse_known_args()
     
    
    LogTrajectory(args)
