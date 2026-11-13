package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * Validation tests for the PathPlanner files in src/main/deploy/pathplanner.
 *
 * <p>Motivation: issue #67 - the robot crashed into the trench barrier at CapCity because the
 * trench swipe paths ended with a non-zero goal end velocity (1.0 m/s instead of 0.0). With
 * residual velocity at the goal, the robot keeps driving after the trajectory ends, straight into
 * whatever is past the last waypoint.
 */
class PathPlannerFilesTest {

    private static final Path DEPLOY_DIR = resolveDeployDir();

    private static Path resolveDeployDir() {
        Path dir = Path.of(System.getProperty("user.dir")).toAbsolutePath();
        for (int i = 0; i < 5 && dir != null; i++) {
            Path candidate = dir.resolve("src/main/deploy/pathplanner");
            if (Files.isDirectory(candidate)) {
                return candidate;
            }
            dir = dir.getParent();
        }
        fail("Could not locate src/main/deploy/pathplanner from " + System.getProperty("user.dir"));
        return null; // unreachable
    }

    private static List<Path> listFiles(String subdir, String extension) throws IOException {
        try (Stream<Path> stream = Files.list(DEPLOY_DIR.resolve(subdir))) {
            return stream.filter(p -> p.getFileName().toString().endsWith(extension)).toList();
        }
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> asObject(Object value, String what) {
        assertTrue(value instanceof Map, what + " should be a JSON object");
        return (Map<String, Object>) value;
    }

    @SuppressWarnings("unchecked")
    private static List<Object> asList(Object value, String what) {
        assertTrue(value instanceof List, what + " should be a JSON array");
        return (List<Object>) value;
    }

    private static double asNumber(Object value, String what) {
        assertTrue(value instanceof Number, what + " should be a JSON number");
        return ((Number) value).doubleValue();
    }

    /** Every .path and .auto file must be valid JSON (a corrupted file fails at robot startup). */
    @Test
    void allPathPlannerFilesParse() throws IOException {
        List<Path> files = new ArrayList<>();
        files.addAll(listFiles("paths", ".path"));
        files.addAll(listFiles("autos", ".auto"));
        assertTrue(!files.isEmpty(), "expected at least one PathPlanner file");
        for (Path file : files) {
            try {
                Json.parse(Files.readString(file));
            } catch (RuntimeException e) {
                fail(file.getFileName() + " is not valid JSON: " + e.getMessage());
            }
        }
    }

    /**
     * Regression guard for issue #67: every path must end at 0 m/s so the robot actually stops at
     * the goal instead of coasting into field elements. If a future path truly needs a non-zero end
     * velocity (e.g. a path meant to chain into a second path file), this test is the place to
     * document that exception.
     */
    @Test
    void everyPathEndsAtZeroVelocity() throws IOException {
        for (Path file : listFiles("paths", ".path")) {
            Map<String, Object> path = asObject(Json.parse(Files.readString(file)), "path file");
            Map<String, Object> goalEndState = asObject(path.get("goalEndState"), "goalEndState");
            double endVelocity = asNumber(goalEndState.get("velocity"), "goalEndState.velocity");
            assertEquals(
                    0.0,
                    endVelocity,
                    file.getFileName()
                            + " ends at "
                            + endVelocity
                            + " m/s; a non-zero goal end velocity leaves the robot driving"
                            + " past the end of its trajectory (issue #67)");
        }
    }

    /** Every path referenced by an auto must exist on disk. */
    @Test
    void autosReferenceExistingPaths() throws IOException {
        for (Path file : listFiles("autos", ".auto")) {
            Map<String, Object> auto = asObject(Json.parse(Files.readString(file)), "auto file");
            List<String> pathNames = new ArrayList<>();
            collectCommandInfo(
                    asObject(auto.get("command"), "auto command"), pathNames, new ArrayList<>());
            for (String pathName : pathNames) {
                assertTrue(
                        Files.isRegularFile(
                                DEPLOY_DIR.resolve("paths").resolve(pathName + ".path")),
                        file.getFileName() + " references missing path '" + pathName + "'");
            }
        }
    }

    /**
     * Every named command used by an auto (or a path event marker) must be registered in
     * RobotContainer, otherwise the auto dies at runtime with an unregistered-command error.
     */
    @Test
    void autosReferenceRegisteredNamedCommands() throws IOException {
        Path robotContainer =
                DEPLOY_DIR.resolve("../../java/frc/robot/RobotContainer.java").normalize();
        String robotContainerSource = Files.readString(robotContainer);

        List<String> namedCommands = new ArrayList<>();
        for (Path file : listFiles("autos", ".auto")) {
            Map<String, Object> auto = asObject(Json.parse(Files.readString(file)), "auto file");
            collectCommandInfo(
                    asObject(auto.get("command"), "auto command"),
                    new ArrayList<>(),
                    namedCommands);
        }
        for (Path file : listFiles("paths", ".path")) {
            Map<String, Object> path = asObject(Json.parse(Files.readString(file)), "path file");
            for (Object marker : asList(path.get("eventMarkers"), "eventMarkers")) {
                Map<String, Object> eventMarker = asObject(marker, "event marker");
                collectCommandInfo(
                        asObject(eventMarker.get("command"), "event marker command"),
                        new ArrayList<>(),
                        namedCommands);
            }
        }
        for (String name : namedCommands) {
            assertTrue(
                    robotContainerSource.contains("NamedCommands.registerCommand(\"" + name + "\""),
                    "named command '" + name + "' is used by an auto but never registered");
        }
    }

    /** Walks a PathPlanner command tree, collecting referenced path names and named commands. */
    @SuppressWarnings("unchecked")
    private static void collectCommandInfo(
            Map<String, Object> command, List<String> pathNames, List<String> namedCommands) {
        if (command == null) {
            return;
        }
        String type = (String) command.get("type");
        Map<String, Object> data = asObject(command.get("data"), "command data");
        switch (type) {
            case "path" -> pathNames.add((String) data.get("pathName"));
            case "named" -> namedCommands.add((String) data.get("name"));
            case "sequential", "parallel", "race", "deadline" -> {
                for (Object child : asList(data.get("commands"), "child commands")) {
                    collectCommandInfo((Map<String, Object>) child, pathNames, namedCommands);
                }
            }
            default -> {
                // wait commands and anything new PathPlanner adds carry no references
            }
        }
    }

    /**
     * Minimal JSON parser so these tests need no third-party dependency. Supports objects, arrays,
     * strings (with escapes), numbers, booleans and null - everything PathPlanner writes.
     */
    private static final class Json {
        private final String text;
        private int pos;

        private Json(String text) {
            this.text = text;
        }

        static Object parse(String text) {
            Json json = new Json(text);
            Object value = json.readValue();
            json.skipWhitespace();
            if (json.pos != json.text.length()) {
                throw new IllegalArgumentException("trailing content at offset " + json.pos);
            }
            return value;
        }

        private void skipWhitespace() {
            while (pos < text.length() && Character.isWhitespace(text.charAt(pos))) {
                pos++;
            }
        }

        private char expect(char c) {
            skipWhitespace();
            if (pos >= text.length() || text.charAt(pos) != c) {
                throw new IllegalArgumentException("expected '" + c + "' at offset " + pos);
            }
            return text.charAt(pos++);
        }

        private Object readValue() {
            skipWhitespace();
            if (pos >= text.length()) {
                throw new IllegalArgumentException("unexpected end of input");
            }
            char c = text.charAt(pos);
            return switch (c) {
                case '{' -> readObject();
                case '[' -> readArray();
                case '"' -> readString();
                case 't' -> readLiteral("true", Boolean.TRUE);
                case 'f' -> readLiteral("false", Boolean.FALSE);
                case 'n' -> readLiteral("null", null);
                default -> readNumber();
            };
        }

        private Map<String, Object> readObject() {
            Map<String, Object> map = new LinkedHashMap<>();
            expect('{');
            skipWhitespace();
            if (pos < text.length() && text.charAt(pos) == '}') {
                pos++;
                return map;
            }
            while (true) {
                String key = (String) readValue();
                expect(':');
                map.put(key, readValue());
                skipWhitespace();
                char c = text.charAt(pos++);
                if (c == '}') {
                    return map;
                }
                if (c != ',') {
                    throw new IllegalArgumentException(
                            "expected ',' or '}' at offset " + (pos - 1));
                }
            }
        }

        private List<Object> readArray() {
            List<Object> list = new ArrayList<>();
            expect('[');
            skipWhitespace();
            if (pos < text.length() && text.charAt(pos) == ']') {
                pos++;
                return list;
            }
            while (true) {
                list.add(readValue());
                skipWhitespace();
                char c = text.charAt(pos++);
                if (c == ']') {
                    return list;
                }
                if (c != ',') {
                    throw new IllegalArgumentException(
                            "expected ',' or ']' at offset " + (pos - 1));
                }
            }
        }

        private String readString() {
            StringBuilder sb = new StringBuilder();
            expect('"');
            while (true) {
                if (pos >= text.length()) {
                    throw new IllegalArgumentException("unterminated string");
                }
                char c = text.charAt(pos++);
                if (c == '"') {
                    return sb.toString();
                }
                if (c == '\\') {
                    char esc = text.charAt(pos++);
                    switch (esc) {
                        case '"' -> sb.append('"');
                        case '\\' -> sb.append('\\');
                        case '/' -> sb.append('/');
                        case 'b' -> sb.append('\b');
                        case 'f' -> sb.append('\f');
                        case 'n' -> sb.append('\n');
                        case 'r' -> sb.append('\r');
                        case 't' -> sb.append('\t');
                        case 'u' -> {
                            sb.append((char) Integer.parseInt(text.substring(pos, pos + 4), 16));
                            pos += 4;
                        }
                        default -> throw new IllegalArgumentException("bad escape \\" + esc);
                    }
                } else {
                    sb.append(c);
                }
            }
        }

        private Object readLiteral(String literal, Object value) {
            if (!text.startsWith(literal, pos)) {
                throw new IllegalArgumentException("bad literal at offset " + pos);
            }
            pos += literal.length();
            return value;
        }

        private Number readNumber() {
            int start = pos;
            while (pos < text.length() && "-+0123456789.eE".indexOf(text.charAt(pos)) >= 0) {
                pos++;
            }
            if (start == pos) {
                throw new IllegalArgumentException("expected a value at offset " + pos);
            }
            return Double.parseDouble(text.substring(start, pos));
        }
    }
}
