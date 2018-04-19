# Particle

## Cloning this repository
$ git clone https://github.com/piyushagade/Particle

## Pushing updates/changes to remote repo
$ git add . <br>
$ git commit -m "ENTER A COMMIT MESSAGE IN PRESENT TENSE"<br>
$ git push -u origin master

## Pulling update from remote repo to local repo
Before a pullo, make sure you are on the right branch ('master' branch). To do this, enter the following command in command-line:

$ git branch

The checked out branch will have a * before the name.

If required, switch to 'master' branch using: <br>
$ git checkout master

Once on master, we are ready to pull updates: <br>
$ git pull origin master

Now you are all up to date with the remote repo.


## Branching
Branches are used to develop features isolated from each other. The master branch is the "default" branch when you create a repository. Use other branches for development and merge them back to the master branch upon completion.

To create a new branch: <br>
$ git checkout -b awesome_feature

To delete a branch: <br>
$ git branch -d bad_feature

To switch to a branch: <br>
$ git checkout new_feature

A branch is not available to others unless you push the branch to your remote repository: <br>
$ git push origin awesome_feature

To merge a branch into the active branch: <br>
$ git merge awesome_feature

Before merging changes, you can also preview them by using (here master is your target branch, and new_feature is the source branch): <br>
$ git diff new_feature master

## Reset and discard local changes
If you want to drop all your local changes and commits, fetch the latest history from the server and point your local master branch at it: <br>
git fetch origin
git reset --hard orgin/master


A good learning resource: <br>
http://rogerdudler.github.io/git-guide/
