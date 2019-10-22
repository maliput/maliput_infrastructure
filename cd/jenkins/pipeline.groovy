withEnv(["COLCON_BUILD_EXTRA_ARGS=${env.COLCON_BUILD_EXTRA_ARGS ?: ''} --cmake-args -DBUILD_DOCS=1"]) {
  load './index/ci/jenkins/pipeline.groovy'
}
stage('deploy') {
  sh './index/cd/jenkins/publish_docs'
}
