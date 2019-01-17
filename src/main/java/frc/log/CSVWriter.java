import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class CSVWriter {
    BufferedWriter writer;
    public CSVWriter(String path) throws IOException {
        File file = new File(path);
        file.createNewFile();
        writer = new BufferedWriter(new FileWriter(file));
    }

    public void newRow(String[] columns) throws IOException {
        String row = String.join(", ", columns);
        writer.write(row);
        writer.newLine();
    }

    public void close() throws IOException {
        writer.close();
    }

    public static void main(String args[]) throws IOException {
        CSVWriter csv = new CSVWriter("test");
        String[] row = { "Value 1", "Value 2", "Test" };
        csv.newRow(row);
        csv.newRow(row);
        csv.close();
    }
}