repoURL = 'https://github.com/frc-2175/2018RobotCode'
jdk = 'jdk1.8.0_111'
slackChannel = '#code'

void setBuildStatus(String message, String state) {
  step([
      $class: 'GitHubCommitStatusSetter',
      reposSource: [$class: 'ManuallyEnteredRepositorySource', url: repoURL],
      errorHandlers: [[$class: 'ChangingBuildStatusErrorHandler', result: 'SUCCESS']],
      statusResultSource: [ $class: 'ConditionalStatusResultSource', results: [[$class: 'AnyBuildResult', message: message, state: state]] ]
  ])
}

int matchInt(String contents, String attributeName) {
  def matcher = contents =~ attributeName + '="([^"]+)"'
  def value = matcher ? matcher[0][1] : null
  if (value != null) {
    return value.toInteger()
  } else {
    return 0
  }
}

String getProjectName() {
  return env.JOB_NAME.tokenize('/')[0]
}

String urlSanitize(String str) {
  str = str.replaceAll("\\s", "_")
  str = str.replaceAll("\\(", "")
  str = str.replaceAll("\\)", "")

  return str
}

node {
  withEnv(["JAVA_HOME=C:\\Users\\Public\\frc2019\\jdk"]) {
    int testCount = 0
    int failureCount = 0
    int skippedCount = 0
    boolean compileSuccess = true
    boolean deploySuccess = true
    boolean startupSuccess = true

    setBuildStatus("Build #${env.BUILD_NUMBER} in progress", 'PENDING')

    stage ('Checkout') {
      checkout scm
    }
    stage ('Build') {
      try {
        bat 'gradlew build'
      } catch (Exception e) {
        currentBuild.result = 'ERROR'
        compileSuccess = false
      }
    }
    if (compileSuccess) {
      stage ('Build and Test') {
        try {
          bat 'gradlew build'
        } catch (Exception e) {
          currentBuild.result = 'ERROR'
        }
      }
      if (failureCount == 0) {
        stage ('Deploy') {
          try {
            bat 'gradlew deploy'
          } catch (Exception e) {
            currentBuild.result = 'ERROR'
            deploySuccess = false
          }
        }
        if (deploySuccess) {
          stage ('Startup') {
            try {
              bat 'ant compile-listener'

              echo '--------------------------\nStarting practice robot\n--------------------------'
              bat 'deployPropertiesFiles_Practice.bat'
              timeout (time: 30, unit: 'SECONDS') {
                bat 'java -jar buildlistener\\listener.jar'
              }
              echo 'Practice robot started up successfully!\n'

              echo '--------------------------\nStarting competition robot\n--------------------------'
              bat 'deployPropertiesFiles_Competition.bat'
              timeout (time: 30, unit: 'SECONDS') {
                bat 'java -jar buildlistener\\listener.jar'
              }
              echo 'Competition robot started up successfully!\n'
            } catch (Exception e) {
              currentBuild.result = 'ERROR'
              startupSuccess = false
            }
          }
        }
      }
    }
    stage ('Update GitHub Status & Notify') {
      boolean overallSuccess = (compileSuccess && failureCount == 0 && deploySuccess && startupSuccess)
      
      def githubStatusMessage = "Compile ${compileSuccess ? 'succeeded' : 'failed'}"
      if (compileSuccess) {
        githubStatusMessage += ", ${testCount - failureCount}/${testCount} tests passed"
        if (failureCount == 0) {
          githubStatusMessage += ", deploy ${deploySuccess ? 'succeeded' : 'failed'}"
          if (deploySuccess) {
            githubStatusMessage += ", startup ${startupSuccess ? 'succeeded' : 'failed'}"
          }
        }
      }
      githubStatusMessage += '.'
      
      setBuildStatus(githubStatusMessage, overallSuccess ? 'SUCCESS' : 'FAILURE')
      
      if (env.BRANCH_NAME == 'master') {
        def slackMessage = "Build #${env.BUILD_NUMBER} ${overallSuccess ? 'Success' : 'Failure'}"
        slackMessage += "\nCompile Result:\n    ${compileSuccess ? 'Success' : 'Failure'}"

        if (compileSuccess) {
          slackMessage += "\nTest Status:\n    Passed: ${testCount - failureCount}, Failed: ${failureCount}, Skipped: ${skippedCount}"
          if (failureCount == 0) {
            slackMessage += "\nDeploy Result:\n    ${deploySuccess ? 'Success' : 'Failure'}"
            if (deploySuccess) {
              slackMessage += "\nStartup Result:\n    ${startupSuccess ? 'Success' : 'Failure'}"
            }
          }
        }

        def buildUrl = urlSanitize("http://fightingcalculators.org/ci/${getProjectName()}/${env.BRANCH_NAME}/${env.BUILD_NUMBER}")
        slackMessage += "\n\nLink: ${buildUrl}"
        
        slackSend channel: slackChannel, color: (overallSuccess ? 'good' : 'danger'), message: slackMessage.trim()
      }
    }
  }
}
