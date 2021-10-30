@echo off

set /p id=Enter NAME for commit (Version X.Y) (X - Current Gate , Y - Increment from previous commit in current Gate): 

set /p branch=Enter branch for pull request (You first name): 

git commit -a -m "%id%"
git push -u origin master:%branch%
git request-pull master:%branch% https://github.com/turtlebrosforever/Gallagher-Station-Duke-Energy-Museum-Exhibit.git



echo "Please check Github to see if this was successful."
echo "If you want your code reviewed by Elliot, please create a pull request via Github."

pause