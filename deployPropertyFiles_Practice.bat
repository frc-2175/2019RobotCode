::Shared Files
echo y | pscp -pw "" src\Properties\*.properties admin@10.21.75.2:/home/lvuser
::Bot Specified Files
echo y | pscp -pw "" src\Properties\practiceBot\*.properties admin@10.21.75.2:/home/lvuser
::SSH files to robot
echo y | plink -ssh -pw "" admin@roborio-2175-frc.local "mkdir -m 775 -v -p log; . /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r"
