stage('[gcc]setup') {
   sh './index/ci/jenkins/setup_workspace'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/pull_sources'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/install_deps'
}
stage('[gcc]build') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/build'
}
stage('[gcc]test') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/test'
}
