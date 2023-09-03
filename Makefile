SHELL = /bin/bash

.PHONY: help

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help

setup-docs:
	cd docs && yarn

setup-deploy:
	cd deploy && yarn

setup: setup-docs setup-deploy		## install and setup everything for development

build-docs:
	cd docs && yarn build

cdk-deploy-docs:
	cd deploy && yarn cdk deploy --all --require-approval never

deploy: setup build-docs cdk-deploy-docs		## deploy everything

dev:		## start dev server (localhost:3000)
	cd docs && yarn dev
