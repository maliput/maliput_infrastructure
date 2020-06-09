stage('[ubsan]setup') {
   sh './index/ci/jenkins/setup_workspace'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/pull_sources'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/install_deps'
}
stage('[ubsan]undefined_sanitizer') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/undefined_sanitizer'
}
