
# Overview of the ROS Architecture
All the code is in sailbot20-21/sailbot_ws/src/sailbot/sailbot
The path has to be this long unfortunately since we need our git repo, then our ros workspace, then our ros package, then our python package

To ensure you have dependancies installed, run 
```rosdep install -i --from-path src --rosdistro dashing -y``` 
from this directory (sailbot20-21/sailbot_ws)

# Connection Via SSH

### On PC

On PC you need this app to connect to the boat via ssh

http://ftp1.digi.com/support/utilities/changer_digi.zip

Assuming the boat is powered and the ethernet bridge is powered and plugged in, open the changer digi tool and hit search.

The tool should show two bridges. The one with IP ending in 17 is on the boat. The one ending in 20 is what you are connected to.

If you do not see either bridge, try turning off wifi and hitting search again.
If that doesn't solve, try restarting your pc, turing off your firewall, etc.

Once you see both bridges you may be able to ssh, but we've found it wont work unless you follow these steps:
1. double click the ip ending in 20, and change the "target device new parameters" to be IP:192.168.17.21. keep all else the same.
2. Hit apply
3. The hit yes
4. Then double click and change back to 192.168.17.20
5. Hit apply. This should then fail. (If it does, carry on)
6. Hit apply again. This should work.

Now you should be able to shh with putty at sailbot@192.168.17.20

The password is "admin"

### On Mac
<i>this is intended for those running macOS Catalina. you can find further instuctions for your version online by searching "how to manually set your IP address - Mac"</i>

On Mac, you can first connect the powered ethernet bridge to your machine. Your Mac should recognize that a new network connection is available

1. go to your network settings
2. navigate to a network with the name "USB 10/100/1000 LAN"
3. change the "Configure IPv4" setting to "Manually"
4. set the IP address to be anything on the 192.168.17 (Class C) subnet, you may not use address 17 or 20 as those are reserved for the target and the express ethernet bridge you are connected to. For my purposes, I set the IP to 192.168.17.4 as 4 is my favorite number (must be between 0 and 255)
5. set the subnet mask to be "255.255.255.0"
6. then click "apply"
7. test by pinging 192.168.17.17 (the target IP for the express ethernet bridge in the hull)


# Running the Nodes
To run the boat, first connect over SSH, and navigate into this directory (sailbot20-21/sailbot_ws)







