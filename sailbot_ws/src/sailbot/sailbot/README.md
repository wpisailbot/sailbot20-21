
# Overview of the ROS Architecture


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


# Running the Nodes
