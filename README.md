# FRC-8871-Reefscape
This is the official repository for FRC team 8871, Saber Dynamics, 2025 Reefscape code.

# TODO

- Assign laptop numbers
- Add names to GitHub account
- Select a subsystem to join (elevator or pivot)

## Subsystems
### Pivot subsystem
- Write functions
### Elevator subsystem
- Write functions
### Swerve subsystem
- idk

## Commands
### Pivot commands
- PID command
### Elevator commands
- PID command
- Default command
- Default command
### Drive command
- Turn to command
- Default command






# GitHub terms
## Code locations
- repo/repository - A repository is where the code is stored. This could be locally on your device or remotely online.
- local - When something is local, it is saved only on your device.
- remote - When something is remote, it is saved online on GitHub.
- main - The main branch is where all the code will eventually be. It is the main code.
- branch - Branches are offshoots of the code. These are used to test and mess around with code without breaking and changing anything in the main branch.
## Code editing
- commit - A commit saves and records any changes you have made to code. You are also required to write a "commit message" which describes what you did.
- push - A push uploads your changes on your local branch to the remote branch.
- pull - If changes are made to the remote repository, a pull updates your local repository to match it.
- pull request - A pull request is a request to merge the updates you made on your branch to the main branch. If you edit file B, you can request the changes to be added to file A.

# Git commands
## Setting your username and email
git config --global user.name "YOUR NAME HERE"
git config --global user.email "YOUR EMAIL HERE"

## Checking and setting your branch
git branch
- If this command returns main, DO NOT PUSH YOUR CODE. Use the following commands.

git fetch
git switch EXISTING-BRANCH-NAME-HERE
- If the branch you want to push to already exists, use those commands to set where you push to, to that branch. If it doesn't exist and you want to create a new branch, use the following command.

git checkout -b NEW-BRANCH-NAME-HERE
- This command creates a new branch with your desired name and switches you to that branch. You can now safely push your code.

git fetch --prune
- This command removes old, deleted branches


