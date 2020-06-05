#!/usr/bin/env groovy

def pipelines = ['gcc_clang_build', 'address_sanitizer', 'ubsan_tsan_sanitizer', 'scan_build']
def branches = [:]
for ( pipeline in pipelines ) {
  def branchName = pipeline

  branches[branchName] = {
    node('delphyne-linux-bionic-unprovisioned') {
      stage(branchName) {
        try {
          stage('checkout') {
            dir('index') {
              checkout scm
            }
          }
          withEnv(['ENABLE_TSAN=ON']) {
            println('./index/ci/jenkins/pipeline_'+branchName+'.groovy')
            load './index/ci/jenkins/pipeline_'+branchName+'.groovy'
          }
        } finally {
          cleanWs(notFailBuild: true)
        }
      }
    }
  }
}


// Give the branches to Jenkins for parallel execution:
parallel branches
