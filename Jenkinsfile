repoURL = 'https://github.com/frc-2175/2019RobotCode'
slackChannel = '#code-automessages'

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
  env.PATH = "C:\\Users\\Public\\frc2019\\roborio\bin;${env.PATH}"
  env.PATH = "C:\\Users\\Public\\frc2019\\frccode;${env.PATH}"
  env.PATH = "C:\\Users\\Public\\frc2019\\jdk\\bin;${env.PATH}"
  env.PATH = "C:\\Program Files\\PuTTY;${env.PATH}"
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
          bat 'gradlew test'
        } catch (Exception e) {
          currentBuild.result = 'ERROR'
        }
        // step([$class: 'JUnitResultArchiver', testResults: 'buildtest/results/*.xml', allowEmptyResults: true])
        def xmlFiles = findFiles(glob: 'build/test-results/**/*.xml')
        for (int i = 0; i < xmlFiles.length; i++) {
          def file = xmlFiles[i]
          def contents = readFile file.getPath()

          testCount += matchInt(contents, 'tests')
          failureCount += matchInt(contents, 'failures') 
          failureCount += matchInt(contents, 'errors') // errors are treated as failures
          skippedCount += matchInt(contents, 'skipped')
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
              bat 'gradlew :listener:build'

              echo '--------------------------\nStarting practice robot\n--------------------------'
              bat 'deployPropertyFiles_Practice.bat'
              timeout (time: 30, unit: 'SECONDS') {
                bat 'java -jar listener\\build\\libs\\listener.jar'
              }
              echo 'Practice robot started up successfully!\n'

              echo '--------------------------\nStarting competition robot\n--------------------------'
              bat 'deployPropertyFiles_Competition.bat'
              timeout (time: 30, unit: 'SECONDS') {
                bat 'java -jar listener\\build\\libs\\listener.jar'
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
        
        slackSend channel: slackChannel, color: (overallSuccess ? 'good' : 'danger'), message: slackMessage.trim()
      }
    }
  }
}
