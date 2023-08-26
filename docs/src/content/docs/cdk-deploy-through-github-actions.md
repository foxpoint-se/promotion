---
title: CDK deploy through Github Actions
description: A simple CI/CD setup.
tags:
  - AWS CDK
  - Github Actions
---

# CDK deploy through Github Actions

## Create user and stuff in AWS IAM

1. Go to the IAM section in AWS Console.
1. Create a custom policy with the following JSON. Call it something like "CDK_Deploy".

   ```json
   {
     "Version": "2012-10-17",
     "Statement": [
       {
         "Effect": "Allow",
         "Action": ["sts:AssumeRole"],
         "Resource": ["arn:aws:iam::*:role/cdk-*"]
       }
     ]
   }
   ```

1. Create a user group called something like "CDK-Deployers".
1. Create a new user called something like "Github Actions".
1. Add that user to the group created above.
1. Navigate into the user and create new access keys.
1. Save those keys temporarily, since they will only be shown once.

## Set up secrets in Github

1. Go to the settings sections of your organization in Github.
1. Navigate to "Secrets and variables" -> "Actions".
1. Create `AWS_ACCESS_KEY_ID` and `AWS_SECRET_KEY` with values from your AWS user created above.

## Set up a simple Github Actions workflow in your repository

1. In your repo, create a folder structure like `.github/workflows/my-build-workflow.yml`.
1. Add something like the following:

   ```yaml
   name: My build workflow

   on:
     push:
       branches: [main]

   jobs:
     deploy:
       runs-on: ubuntu-latest
       steps:
         - run: echo "The job was automatically triggered by a ${{ github.event_name }} event."
         - run: echo "This job is now running on a ${{ runner.os }} server hosted by GitHub."
         - run: echo "The branch is ${{ github.ref }} and the repository is ${{ github.repository }}."
         - uses: actions/checkout@v3
         - name: Set up Node
           uses: actions/setup-node@v3
           with:
             node-version: "18.x"
             cache: "yarn"
             cache-dependency-path: path/to/yarn.lock
         - name: Configure AWS credentials
           uses: aws-actions/configure-aws-credentials@master
           with:
             aws-access-key-id: ${{ secrets.AWS_ACCESS_KEY_ID }}
             aws-secret-access-key: ${{ secrets.AWS_SECRET_KEY }}
             aws-region: "eu-west-1" # or us-east-1 or whatever.
         - run: make deploy # assuming `make deploy` does everything it should. you can use stuff like `yarn cdk deploy` as well.
   ```

   ## Try it out

   Push a new commit to your repo and follow the job in Github Actions. If everything works well, new commits to your main branch should trigger a deploy in AWS.
