package frc.log;

import java.util.HashMap;

public interface Loggable {
	public String getLogType();

	public String getId();

	public HashMap<String, Object> getValues();
}
