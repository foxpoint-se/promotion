#!/usr/bin/env node
import "source-map-support/register";
import * as cdk from "aws-cdk-lib";
import { DeployStack } from "../lib/deploy-stack";

const app = new cdk.App();
new DeployStack(app, "PromotionSiteStack", {
  env: { account: "485563272586", region: "eu-west-1" },
});
