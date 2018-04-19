#!/bin/sh

#init git-subrepo
git clone `grep git-commands .gitrepos`
if [ ! -x ./git-commands/git-subrepo ];then
  exit
fi

./git-commands/git-subrepo
