using System;
using System.IO;
using System.Text;

namespace SimHubConfigGenerator
{
    class Program
    {
        // Команды из dma_stepper_hal.h
        const byte CMD_HOME = 0;
        const byte CMD_MOVE = 1;
        const byte CMD_SET_SPEED = 2;
        const byte CMD_DISABLE = 3;
        const byte CMD_ENABLE = 4;
        const byte CMD_PARK = 7;
        const byte CMD_SET_ACCEL = 10;

        const int AXES_TOTAL = 6;
        const int PACKET_SIZE = 28; // 4 + 6*4 = 0x1C

        static void Main(string[] args)
        {
            Console.WriteLine("=== SimHub 6DOF Config Generator ===");
            Console.WriteLine("vAzhureRacing 6DOF Motion Controller\n");

            // Параметры (можно менять)
            int speedMmPerSec = 250;
            int accelStepsPerSec2 = 15000;
            string comPort = "COM9";

            if (args.Length >= 1) int.TryParse(args[0], out speedMmPerSec);
            if (args.Length >= 2) int.TryParse(args[1], out accelStepsPerSec2);
            if (args.Length >= 3) comPort = args[2];

            Console.WriteLine($"Speed: {speedMmPerSec} mm/s");
            Console.WriteLine($"Accel: {accelStepsPerSec2} steps/s²");
            Console.WriteLine($"COM Port: {comPort}\n");

            string json = GenerateJson(speedMmPerSec, accelStepsPerSec2, comPort);

            string outFile = "SimHub_6DOF_Config.json";
            File.WriteAllText(outFile, json, Encoding.UTF8);
            Console.WriteLine($"Config saved to: {Path.GetFullPath(outFile)}\n");

            Console.WriteLine("=== Generated Commands ===\n");
            PrintCommands(speedMmPerSec, accelStepsPerSec2);

            Console.WriteLine("\nPress any key to exit...");
            Console.ReadKey();
        }

        static string GenerateJson(int speed, int accel, string comPort)
        {
            string guid1 = Guid.NewGuid().ToString();
            string guid2 = Guid.NewGuid().ToString();

            return $@"{{
  ""Output"": {{
    ""Comments"": null,
    ""CustomName"": ""vAzhureRacing 6DOF"",
    ""Settings"": {{
      ""RtsEnable"": true,
      ""DtrEnable"": true,
      ""LockSettings"": false,
      ""SerialPort"": ""{comPort}"",
      ""DataBits"": 8,
      ""SerialStopBits"": 1,
      ""SerialParity"": 0,
      ""BaudRate"": 115200,
      ""AfterOpenDelay"": 0,
      ""GenericProtocolDefinition"": {{
        ""SettingsBuilder"": {{
          ""Settings"": [],
          ""IsEditMode"": false
        }},
        ""ShowWaitForResponse"": true,
        ""AxisResolution"": 32,
        ""AxisFormat"": 1,
        ""StartCommands"": [
          {{
            ""MustWaitForMessage"": false,
            ""WaitForMessage"": null,
            ""WaitForDelay"": 5000,
            ""Command"": ""{BuildCommand(CMD_SET_SPEED, speed)}"",
            ""CommandDelay"": 100
          }},
          {{
            ""MustWaitForMessage"": false,
            ""WaitForMessage"": null,
            ""WaitForDelay"": 5000,
            ""Command"": ""{BuildCommand(CMD_SET_ACCEL, accel)}"",
            ""CommandDelay"": 100
          }},
          {{
            ""MustWaitForMessage"": false,
            ""WaitForMessage"": null,
            ""WaitForDelay"": 5000,
            ""Command"": ""{BuildCommand(CMD_ENABLE, 1)}"",
            ""CommandDelay"": 3000
          }},
          {{
            ""MustWaitForMessage"": false,
            ""WaitForMessage"": null,
            ""WaitForDelay"": 5000,
            ""Command"": ""{BuildCommand(CMD_HOME, 1)}"",
            ""CommandDelay"": 100
          }}
        ],
        ""UpdateCommands"": [
          {{
            ""Command"": ""<0x00><0x1C><0x0B><0x00><Axis1><Axis2><Axis3><Axis4><Axis5><Axis6>"",
            ""CommandDelay"": 10
          }}
        ],
        ""StopCommands"": [
          {{
            ""MustWaitForMessage"": false,
            ""WaitForMessage"": null,
            ""WaitForDelay"": 5000,
            ""Command"": ""{BuildCommand(CMD_PARK, 1)}"",
            ""CommandDelay"": 1
          }}
        ]
      }},
      ""SecurityAcknowledgementAccepted"": true,
      ""ActuatorOrderingSettings"": {{
        ""MaxActuatorsEx"": 6,
        ""ConfigurationDone"": true,
        ""UseParkPosition"": false,
        ""ParkDuration"": 5516,
        ""UseParkPositionEx"": false,
        ""Roles"": [
          {{ ""RangeLimit"": 100.0, ""ParkPosition"": 50.0, ""Role"": 1, ""ReverseDirection"": false }},
          {{ ""RangeLimit"": 100.0, ""ParkPosition"": 50.0, ""Role"": 3, ""ReverseDirection"": false }},
          {{ ""RangeLimit"": 100.0, ""ParkPosition"": 50.0, ""Role"": 4, ""ReverseDirection"": false }},
          {{ ""RangeLimit"": 100.0, ""ParkPosition"": 50.0, ""Role"": 2, ""ReverseDirection"": false }},
          {{ ""RangeLimit"": 100.0, ""ParkPosition"": 50.0, ""Role"": 5, ""ReverseDirection"": false }},
          {{ ""RangeLimit"": 100.0, ""ParkPosition"": 50.0, ""Role"": 6, ""ReverseDirection"": false }}
        ]
      }},
      ""OutputId"": ""{guid1}"",
      ""AllowIdling"": true
    }},
    ""OutputId"": ""{guid2}"",
    ""TypeName"": ""GenericSerialOutputV2""
  }}
}}";
        }

        static string BuildCommand(byte cmd, int value)
        {
            var sb = new StringBuilder();

            // Header: 0x00
            sb.Append("<0x00>");
            // Length: 0x1C (28)
            sb.Append("<0x1C>");
            // Command
            sb.Append($"<0x{cmd:X2}>");
            // Reserved: 0x00
            sb.Append("<0x00>");

            // Data: 6 × int32_t (little-endian)
            byte[] bytes = BitConverter.GetBytes(value);
            if (!BitConverter.IsLittleEndian)
                Array.Reverse(bytes);

            for (int i = 0; i < AXES_TOTAL; i++)
            {
                foreach (byte b in bytes)
                {
                    sb.Append($"<0x{b:X2}>");
                }
            }

            return sb.ToString();
        }

        static void PrintCommands(int speed, int accel)
        {
            Console.WriteLine($"CMD_SET_SPEED ({speed} mm/s):");
            Console.WriteLine(BuildCommand(CMD_SET_SPEED, speed));

            Console.WriteLine($"\nCMD_SET_ACCEL ({accel} steps/s²):");
            Console.WriteLine(BuildCommand(CMD_SET_ACCEL, accel));

            Console.WriteLine("\nCMD_ENABLE:");
            Console.WriteLine(BuildCommand(CMD_ENABLE, 1));

            Console.WriteLine("\nCMD_HOME:");
            Console.WriteLine(BuildCommand(CMD_HOME, 1));

            Console.WriteLine("\nCMD_MOVE (UpdateCommand):");
            Console.WriteLine("<0x00><0x1C><0x01><0x00><Axis1><Axis2><Axis3><Axis4><Axis5><Axis6>");

            Console.WriteLine("\nCMD_PARK (StopCommand):");
            Console.WriteLine(BuildCommand(CMD_PARK, 1));
        }
    }
}
