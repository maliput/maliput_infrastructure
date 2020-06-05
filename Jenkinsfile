#!/usr/bin/env groovy

def pipelines = ['pipeline_scan_build', 'pipeline_address_sanitizer', 'pipeline_ubsan_tsan_sanitizer', 'pipeline_scan_build']
def branches = [:]
for ( pipeline in pipelines ) {
    def branchName = pipeline

    branches[branchName] = {
        node('delphyne-linux-bionic-unprovisioned') {
            stage(branchName) {
                println('Running ' + branchName)
              stage(branchName + 'inside stage') {
                println('Running ' + branchName)
            }
            }
        }
    }
}

// Give the branches to Jenkins for parallel execution:
parallel branches

node('delphyne-linux-bionic-unprovisioned') {
   println('Finishing work')
}
