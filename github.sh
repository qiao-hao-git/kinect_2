#!/bin/bash

chmod u+x github.sh
git init

git add .
git commit -m "脚本"
git remote add origin git@github.com:qiao-hao-git/kinect_2.git
git push origin master
