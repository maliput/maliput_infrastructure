stage('setup') {
   sh './index/ci/jenkins/setup_workspace'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/pull_sources'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/install_deps'
}
stage('undefined_sanitizer') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/undefined_sanitizer'
}
stage('tear_down_build') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/tear_down_build'
}
stage('thread_sanitizer') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/thread_sanitizer'
}
