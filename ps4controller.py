from pyPS4Controller.controller import Controller, Event
import rospy
from sensor_msgs.msg import Joy
print("correct")
remoteStatePublisher = rospy.Publisher('/remote_state', Joy, queue_size=10)
remote_state=Joy()
axes=[0.0,0.0,0.0,0.0]   #lx ly  rx ry
buttons=[0,0,0,0]        #up down right left
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_up_arrow_press(self):
       print("Hello world")
       buttons[0]=1
       remote_state.header.seq=1
       remote_state.axes=axes
       remote_state.buttons=buttons
       remoteStatePublisher.publish(remote_state);

    def on_up_down_arrow_release(self):
       print("Goodbye world")
       buttons[0]=0
       buttons[1]=0
       remote_state.axes=axes
       remote_state.buttons=buttons
       
       remoteStatePublisher.publish(remote_state);
       
       
       
    def on_down_arrow_press(self):
       print("Hello world")
       buttons[1]=1
       remote_state.header.seq=1;
       remote_state.axes=axes
       remote_state.buttons=buttons
       
       remoteStatePublisher.publish(remote_state);

    def on_right_arrow_press(self):
       print("Goodbye world")
       buttons[2]=1
       remote_state.axes=axes
       remote_state.buttons=buttons
       
       remoteStatePublisher.publish(remote_state);       
       
    def on_left_arrow_press(self):
        buttons[3]=1
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);

    def on_left_right_arrow_release(self):
        
        print("Goodbye world")
        buttons[2]=0
        buttons[3]=0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);

    def on_L3_up(self,x):
        print("l3 up",x/258.0)
        axes[1]=-x/256.0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);
        
        
    def on_L3_down(self,x):
        print("l3 down",x/258.0)
        axes[1]=-x/256.0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);
        
    def on_L3_left(self,x):
        print("l3 left",x/258.0)
        axes[0]=x/256.0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);
        
    def on_L3_right(self,x):
        print("l3 right",x/258.0)
        axes[0]=x/256.0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);
    
    def on_L3_at_rest(self):
        print("l3 rest")
        axes[0]=0
        axes[1]=0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);
        
        
        
    def on_R3_up(self,x):
        print("r3 up",x/258.0)
        axes[3]=-x/256.0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);
        
        
    def on_R3_down(self,x):
        print("r3 down",x/258.0)
        axes[3]=-x/256.0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);
        
    def on_R3_left(self,x):
        print("r3 left",x/258.0)
        axes[2]=x/256.0
        remote_state.axes=axes
        remoteStatePublisher.publish(remote_state);
        
    def on_R3_right(self,x):
        print("r3 right",x/258.0)
        axes[2]=x/256.0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);
    
    def on_R3_at_rest(self):
        print("r3 rest")
        axes[2]=0
        axes[3]=0
        remote_state.axes=axes
        remote_state.buttons=buttons
       
        remoteStatePublisher.publish(remote_state);        

class MyEventDefinition(Event):

    def __init__(self, **kwargs):
        Event.__init__(self, **kwargs)
    
    # each overloaded function, has access to:
    # - self.button_id
    # - self.button_type
    # - self.value
    # use those variables to determine which button is being pressed
    def x_pressed(self):
        return self.button_id == 0 and self.button_type == 1 and self.value == 1

    def x_released(self):
        return self.button_id == 0 and self.button_type == 1 and self.value == 0
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('moveto', anonymous=True)
        controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False, event_definition=MyEventDefinition)
        controller.debug = True  # you will see raw data stream for any button press, even if that button is not mapped
        # you can start listening before controller is paired, as long as you pair it within the timeout window
        controller.listen(timeout=2)
        
            
    except rospy.ROSInterruptException:
        

        pass


