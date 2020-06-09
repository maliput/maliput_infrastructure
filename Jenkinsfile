#!/usr/bin/env groovy

def pipelines = ['gcc', 'clang', 'asan', 'ubsan', 'tsan', 'scan_build']
def branches = [:]

if (env.BRANCH_NAME == 'master' && timeTriggered()) {
  node('delphyne-linux-bionic-unprovisioned') {
    timeout(60) {
      ansiColor('xterm') {
        def triggers = []
        triggers << cron('H H(7-8) * * *')
        properties ([
          pipelineTriggers(triggers)
        ])
        try {
          stage('checkout') {
            dir('index') {
              checkout scm
            }
          }
          load './index/cd/jenkins/pipeline.groovy'
        } finally {
          cleanWs(notFailBuild: true)
        }
      }
    }
  }
} else {
  for ( pipeline in pipelines ) {
    def branchName = pipeline

    branches[branchName] = {
      node('delphyne-linux-bionic-unprovisioned') {
        timeout(60) {
          ansiColor('xterm') {
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
    }
  }
  branches.failFast = true
  // Give the branches to Jenkins for parallel execution:
  parallel branches
}

def timeTriggered() {
  def causes = currentBuild.getBuildCauses()
  for (cause in causes) {
    if (cause._class.toString().contains('TimerTrigger')) {
      return true;
    }
  }
  return false;
}
