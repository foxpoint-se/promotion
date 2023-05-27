SHELL = /bin/bash

.PHONY: help

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help

setup-web:
	cd web && yarn

setup-deploy:
	cd deploy && yarn

setup: setup-web setup-deploy		## install and setup everything for development

build-web:
	cd web && yarn build

cdk-deploy-web:
	cd deploy && yarn cdk deploy

deploy: setup build-web cdk-deploy-web		## deploy everything

web-dev:		## start web dev server (localhost:3000)
	cd web && yarn dev
