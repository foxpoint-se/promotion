import * as cdk from "aws-cdk-lib";
import { Construct } from "constructs";

export class DeployStack extends cdk.Stack {
  constructor(scope: Construct, id: string, props?: cdk.StackProps) {
    super(scope, id, props);

    const siteBucket = new cdk.aws_s3.Bucket(this, "PromotionSiteBucket", {
      publicReadAccess: true,
      removalPolicy: cdk.RemovalPolicy.DESTROY,
      websiteIndexDocument: "index.html",
    });

    const deployment = new cdk.aws_s3_deployment.BucketDeployment(
      this,
      "PromotionSiteDeployment",
      {
        sources: [cdk.aws_s3_deployment.Source.asset("../out")],
        destinationBucket: siteBucket,
      }
    );
  }
}
