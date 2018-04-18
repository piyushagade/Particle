# Particle

## Cloning this repository
$ git clone https://github.com/libgit2/libgit2

## Pushing updates/changes to remote repo
$ git add .
$ git commit -m "ENTER A COMMIT MESSAGE IN PRESENT TENSE"
$ git push -u origin master

## Pulling update from remote repo to local repo
Before a pullo, make sure you are on the right branch ('master' branch). To do this, enter the following command in command-line:

$ git branch

The checked out branch will have a * before the name.

If required, switch to 'master' branch using:
$ git checkout master

Once on master, we are ready to pull updates:
$ git pull origin master

Now you are all up to date with the remote repo.