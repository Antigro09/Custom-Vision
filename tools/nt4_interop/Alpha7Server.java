import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.Base64;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.StringSubscriber;
import org.wpilib.networktables.TimestampedString;

/** Camera-free alpha-7 NT4 server. No robot, HAL, scheduler, or hardware API. */
public final class Alpha7Server {
  private static final String PREFIX = "CV_INTEROP\t";

  private static void value(String event, TimestampedString value) {
    String encoded = Base64.getEncoder().encodeToString(value.value.getBytes(StandardCharsets.UTF_8));
    System.out.println(PREFIX + event + "\t" + value.timestamp + "\t" + value.serverTime
        + "\t" + NetworkTablesJNI.now() + "\t" + encoded);
    System.out.flush();
  }

  private static void state(String event, NetworkTableInstance instance) {
    System.out.println(PREFIX + event + "\t" + instance.isConnected() + "\t"
        + NetworkTablesJNI.now());
    System.out.flush();
  }

  public static void main(String[] args) throws Exception {
    if (args.length != 3) {
      throw new IllegalArgumentException("Expected port, result topic, cache persistence path");
    }
    int port = Integer.parseInt(args[0]);
    if (port < 1024 || port > 65535) throw new IllegalArgumentException("Unprivileged port required");
    String topicName = args[1];
    String persistencePath = args[2];
    // Independent upper bound in addition to Python's subprocess timeouts.
    Thread watchdog = new Thread(() -> {
      try { Thread.sleep(45000); } catch (InterruptedException ignored) { return; }
      System.exit(3);
    }, "interop-time-limit");
    watchdog.setDaemon(true);
    watchdog.start();
    try (NetworkTableInstance instance = NetworkTableInstance.create();
         StringSubscriber subscriber = instance.getStringTopic(topicName).subscribe("",
             PubSubOption.periodic(0.01), PubSubOption.SEND_ALL,
             PubSubOption.KEEP_DUPLICATES, PubSubOption.pollStorage(64));
         BufferedReader input = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8))) {
      // Bind only localhost. Empty mDNS service disables service advertising.
      instance.startServer(persistencePath, "127.0.0.1", "", port);
      state("READY", instance);
      for (String command; (command = input.readLine()) != null;) {
        switch (command) {
          case "SNAPSHOT" -> value("SNAPSHOT", subscriber.getAtomic());
          case "QUEUE" -> {
            for (TimestampedString item : subscriber.readQueue()) value("ITEM", item);
            state("QUEUE_END", instance);
          }
          case "STATUS" -> state("STATUS", instance);
          case "RETAIN" -> {
            instance.getTopic(topicName).setRetained(true);
            state("RETAIN", instance);
          }
          case "LATE" -> {
            try (StringSubscriber late = instance.getStringTopic(topicName).subscribe("")) {
              value("LATE", late.getAtomic());
            }
          }
          case "STOP" -> { instance.stopServer(); state("STOP", instance); }
          case "START" -> {
            instance.startServer(persistencePath, "127.0.0.1", "", port);
            state("START", instance);
          }
          case "CLOSE" -> { instance.stopServer(); state("CLOSE", instance); return; }
          default -> throw new IllegalArgumentException("Unknown harness command");
        }
      }
    }
  }
}
