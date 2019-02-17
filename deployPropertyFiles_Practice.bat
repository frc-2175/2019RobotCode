::Shared Files
echo y | pscp -pw "" src\Properties\*.properties admin@roborio-2175-frc.local:/home/lvuser
::Bot Specified Files
echo y | pscp -pw "" src\Properties\practiceBot\*.properties admin@roborio-2175-frc.local:/home/lvuser
::SSH files to robot
echo y | plink -ssh -pw "" admin@roborio-2175-frc.local "mkdir -m 775 -v -p log; . /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r"
