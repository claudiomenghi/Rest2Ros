# Rest2Ros

It converts REST messages into ROS messages

It can be used for example in combination with
[PsAlMISt: Pattern bAsed MIssion Specifier](https://github.com/claudiomenghi/PsAlMISt) (a Java tool allows the specification of robotic missions)

### Usage

* Clone the repository in your catkin workspace <br/>
```git clone https://github.com/claudiomenghi/Rest2Ros.git```

* Compile your carkin workspace<br/>
```catkin_make```

* Source the workspace<br/>
```source devel/setup.bash```

* Run the `Rest2Ros` component<br/>
```rosrun ugot communication_manager.py```

