# Byte_Quadruped_robot
Byte Researching project from ITESM GDL

# For new repository on command line:

echo "# test" >> README.md 
<br />
git init   <br />
git add README.md  <br />
git commit -m "First commit"  <br /> 
git remote add origin master https://https://github.com/Sam-ValCor/Byte_Quadruped_robot.git  <br />
git push -u origin master  <br />
# Here are several python codes and two .xml algorithm files for Byte used.
* Cam_Test.py ( An individual test to detect faces and visualize its behaviour.)
*constants.py (Those are the constants defined for servo motors, values for distance between servo links, among others.)
*haarcascade_eye & haarcascade_frontalface_default .xml (Those are the algorithm to detect faces). -I recommend improving it. 
*ik_cal_main.py (The  main routine to calculate the Inverse Kinematic (IK) for Byte)
*ik_leg.py (The routine for the IK for legs that use the fixed distance for the servo Links)
* my_map.py (A map routine to linearized values)
*rot_trans_dh.py (Are the rotation and translation Denavith Hartenberg matrixes calculated for Byte)
*servo_identi_test (It is an individual script to test the servos in an individual way).
*servos_functions.py (Those are the function to generate specific movements as up - down - home )
*Servos_test.py (This Script is just to run the functions of the servos for bite movements [up -down -home "Lagartijas"]
*main.py (This is the main code programmed with the aim to implement the visual detection and the inverse kinematic...
... that contains the real implementation for the tracking. It has a commented part which is the IK tracking, but it requires the modifying and calculating its complement angles)
