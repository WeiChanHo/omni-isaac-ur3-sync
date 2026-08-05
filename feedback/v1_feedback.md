v1_modification:

[X] 1. duration time 的設定 還須要再調整 ，應該改為 不能給使用設定，應該就全部給系統自己計算要運動的時間，能夠設定的應該要是speed ,we could have a sidebar and we could drag it to control the speed , and the range should be set up carefully (minimum and the maximum should be all work correctly even consider to real arm's limit,could refer to feedback/ur3_us.pdf)

[X] 2. while the trajectory executing ,if it stop because the real arm hit something,and it have not reached the goal,we should have a pop up message to warn the user and stop the execution and update the canceled result 


Next step : 
[] 1. current mode: nomatter which way we controll the arm in simulation (robotic poser or physic inspector),we could could access the current joint state and let the user could choose the pose with <current> ,then the real arm update to the current pose in sim (do not need any saved pose)

next next step:
[] 2. livestream : enhance the update frequency, then the sim arm and the real arm will synchronized with each other (truely digital twins)
