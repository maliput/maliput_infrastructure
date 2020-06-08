stage('[clang]setup') {
   sh './index/ci/jenkins/setup_workspace'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/pull_sources'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/install_deps'
}
stage('[clang]build') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/clang_build'
}
stage('[clang]test') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/test'
}
