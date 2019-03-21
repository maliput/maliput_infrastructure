#!/usr/bin/env groovy

node('dsim-linux-bionic-unprovisioned') {
  // From empirical evidence it takes ~10 minutes to install dependencies
  // and ~20 minutes to build and run the tests.  That adds up to 30 minutes
  // which we double to 60 to give us enough leeway.
  timeout(60) {
    ansiColor('xterm') {
      try {
        stage('checkout') {
          dir('index') {
             checkout scm
          }
        }
        load './index/ci/jenkins/pipeline.groovy'
      } finally {
        cleanWs(notFailBuild: true)
      }
    }
  }
}
