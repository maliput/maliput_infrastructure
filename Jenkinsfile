#!/usr/bin/env groovy

def pipelines = ['gcc', 'clang', 'asan', 'ubsan', 'tsan']
def branches = [:]
for ( pipeline in pipelines ) {
  def branchName = pipeline

  branches[branchName] = {
    node('delphyne-linux-bionic-unprovisioned') {
      try {
        stage('[' + branchName + ']' + 'checkout') {
          dir('index') {
            checkout scm
          }
        }
        withEnv(['ENABLE_TSAN=ON']) {
          load './index/ci/jenkins/pipeline_' + branchName + '.groovy'
        }
      } finally {
        cleanWs(notFailBuild: true)
      }
    }
  }
}

branches.failFast = true
// Give the branches to Jenkins for parallel execution:
parallel branches
