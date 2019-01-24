package frc.log;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.ServiceLocator;

public class RobotLogger {
	private ArrayList<Loggable> loggers;
	private final static Logger log = Logger.getLogger(RobotLogger.class.getName());
	public final static String BASE_DIRECTORY = "/home/lvuser/log/";
	private int matchNumber = 0;
	private final static int NUMBER_OF_FOLDERS_TO_KEEP = 10;

	private HashMap<Loggable, CSVWriter> writers;

	public RobotLogger() {
		File workingDirectory = new File(BASE_DIRECTORY);
		workingDirectory.mkdirs();

		writers = new HashMap<>();

		ServiceLocator.register(this);

		loggers = new ArrayList<>();

		File[] listedFiles = workingDirectory.listFiles();
		if (listedFiles == null) {
			log.log(Level.SEVERE, "Could not find directory at '" + BASE_DIRECTORY + "'");
		}

		for (File file : listedFiles) {
			if (file.isDirectory()) {
				try {
					int current = Integer.parseInt(file.getName());
					if (current > matchNumber) {
						matchNumber = current;
					}
				} catch (NumberFormatException e) {
					log.log(Level.WARNING, "Folder parsed did not contain an integer", e);
				}
			}
		}
		matchNumber++;
		new File(BASE_DIRECTORY + matchNumber).mkdirs();

		listedFiles = workingDirectory.listFiles();
		if (listedFiles == null) {
			log.log(Level.SEVERE, "Could not find directory at '" + BASE_DIRECTORY + "'");
		}

		Arrays.sort(listedFiles, (File fileOne, File fileTwo) -> {
			try {
				int fileOneNumber = Integer.parseInt(fileOne.getName());
				int fileTwoNumber = Integer.parseInt(fileTwo.getName());
				int result;
				if (fileOneNumber > fileTwoNumber) {
					result = 1;
				} else if (fileOneNumber < fileTwoNumber) {
					result = -1;
				} else {
					result = 0;
				}
				return result;
			} catch (NumberFormatException e) {
				log.log(Level.WARNING, "Failed to parse int from folder name", e);
				return 0;
			}
		});
		for (int i = 0; i < listedFiles.length - NUMBER_OF_FOLDERS_TO_KEEP; i++) {
			File[] directoryContents = listedFiles[i].listFiles();
			if (directoryContents != null) {
				for (File file : directoryContents) {
					file.delete();
				}
			}
			listedFiles[i].delete();
		}
	}

	public void addLoggable(Loggable loggable) {
		loggers.add(loggable);
	}

	public String getLogFilename(Loggable l) {
		return BASE_DIRECTORY + "/" + matchNumber + "/" + l.getId().replace(" ", "") + ".csv";
	}

	public void logLoggable(Loggable l) throws IOException {
		CSVWriter w = writers.get(l);
		if (w == null) {
			w = new CSVWriter(getLogFilename(l));
			writers.put(l, w);
			ArrayList<String> keys = new ArrayList<>();
			keys.add("Timestamp");
			for(String key : l.getValues().keySet()) {
				keys.add(key);
			}
			w.newRow(Arrays.copyOf(keys.toArray(), keys.toArray().length, String[].class));
		}

		ArrayList<String> row = new ArrayList<>();
		row.add(Double.toString(Timer.getFPGATimestamp()));
		for(Object value : l.getValues().values()) {
			row.add(value.toString());
		}

		w.newRow(Arrays.copyOf(row.toArray(), row.toArray().length, String[].class));
	}

	public void flush() {
		for(CSVWriter w : writers.values()) {
			try {
				w.flush();
			} catch(Exception e) {
				System.out.println("Failed to flush (the toilet's clogged)");
			}
		}
	}

	public void log() {
		for (Loggable logger : loggers) {
			try {
				logLoggable(logger);
			} catch (IOException e) {
				log.log(Level.SEVERE, "Failed to log " + getLogFilename(logger), e);
			}
		}
	}

	public void moveLogFile() {
		try {
			Files.copy(Paths.get("/home/lvuser/logTemp/frc2175.log"),
				Paths.get(BASE_DIRECTORY + "/" + matchNumber + "/event.log"), StandardCopyOption.REPLACE_EXISTING);
		} catch (IOException e) {
			log.log(Level.SEVERE, "Could not copy log files over to log viewer directory!!!", e);
		}
	}

	public static <T> Logger getLogger(Class<T> clazz) {
		return Logger.getLogger(clazz.getName());
	}

	public static Logger getLogger(Object obj) {
		return Logger.getLogger(obj.getClass().getName());
	}

}
