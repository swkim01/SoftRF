/*
 * LogHelper.cpp
 * Copyright (C) 2016-2020 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "LogHelper.h"
#ifndef SDGPXLOG
#include "SoCHelper.h"
#endif

#if LOGGER_IS_ENABLED

#ifdef SDGPXLOG
#include "EEPROMHelper.h"
#include "GNSSHelper.h"
//#include <SD_MMC.h>
//#define SD SD_MMC
#include <mySD.h>
#include <TimeLib.h>

#define PIN_SD_CS 13
#define PIN_SD_MOSI 15
#define PIN_SD_MISO 2
#define PIN_SD_CLK 14

// Seek to fileSize + this position before writing track points.
#define SEEK_TRKPT_BACKWARDS -24
#define GPX_EPILOGUE "\t</trkseg></trk>\n</gpx>\n"
#define LATLON_PREC 6

int TimeZone = 9;
File sd;
File gpxFile;
int sd_avail;
static unsigned int prevTime;

// General-purpose text buffer used in formatting.
char buf[32];

static void openTimestampedFile(const char *shortSuffix, File &file) {
  sprintf(
      buf,
      "/%02d%02d%02d/%02d%02d%02d%s",
      gnss.date.year(),
      gnss.date.month(),
      gnss.date.day(),
      gnss.time.hour()+TimeZone,
      gnss.time.minute(),
      gnss.time.second(),
      shortSuffix);
  Serial.print(F("Starting file "));
  Serial.println(buf);
  if (SD.exists(buf)) {
    Serial.println(F("warning: already exists, overwriting."));
  }
  gpxFile = SD.open(buf, FILE_WRITE);
  if (!gpxFile) {
    Serial.println(F("error: cannot create new file!"));
  }
}

static void startFilesOnSdNoSync() {
  // directory
  sprintf(
      buf,
      "/%02d%02d%02d",
      gnss.date.year(),
      gnss.date.month(),
      gnss.date.day());
  if (!SD.exists(buf)) {
    if (!SD.mkdir(buf)) {
      Serial.println("error: Creating log directory for today failed.");
    }
  }

  // SdFat will silently die if given a filename longer than "8.3"
  // (8 characters, a dot, and 3 file-extension characters).

  // GPX log
  openTimestampedFile(".gpx", gpxFile);
  gpxFile.print(F(
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<gpx version=\"1.0\">\n"
    "\t<trk><trkseg>\n"));
  gpxFile.print(F(GPX_EPILOGUE));
}

static void writeFormattedSampleDatetime(File &file) {
  sprintf(
      buf,
      "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
      gnss.date.year(),
      gnss.date.month(),
      gnss.date.day(),
      gnss.time.hour()+TimeZone,
      gnss.time.minute(),
      gnss.time.second(),
      gnss.time.centisecond());
  file.print(buf);
}

static char * trim(char *buf) {
  while (*buf == ' ' || *buf == '\t') buf++;
  return buf;
}

static void writeGpxSampleToSd() {
  char buf[12];
  gpxFile.seek(gpxFile.size() + SEEK_TRKPT_BACKWARDS);
  gpxFile.print(F("\t\t<trkpt "));

  gpxFile.print(F("lat=\""));
  gpxFile.print(trim(dtostrf(gnss.location.lat(), 10, LATLON_PREC, buf)));
  gpxFile.print(F("\" lon=\""));
  gpxFile.print(trim(dtostrf(gnss.location.lng(), 10, LATLON_PREC, buf)));
  gpxFile.print(F("\">"));

  gpxFile.print(F("<time>"));
  writeFormattedSampleDatetime(gpxFile);
  gpxFile.print(F("</time>"));

  if (gnss.altitude.isValid()) {
    gpxFile.print(F("<ele>")); // meters
    gpxFile.print(trim(dtostrf(gnss.altitude.value(), 5, 2 /* centimeter precision */, buf)));
    gpxFile.print(F("</ele>"));
  }

  if (gnss.speed.isValid()) {
    gpxFile.print(F("<speed>"));
    gpxFile.print(trim(dtostrf(gnss.speed.value(), 5, 1, buf)));
    gpxFile.print(F("</speed>"));
  }
  if (gnss.course.isValid()) {
    gpxFile.print(F("<course>"));
    gpxFile.print(trim(dtostrf(gnss.course.value(), 5, 1, buf)));
    gpxFile.print(F("</course>"));
  }

  if (gnss.satellites.isValid()) {
    gpxFile.print(F("<sat>"));
    gpxFile.print(gnss.satellites.value());
    gpxFile.print(F("</sat>"));
  }
  if (gnss.hdop.isValid()) {
    gpxFile.print(F("<hdop>"));
    gpxFile.print(trim(dtostrf(gnss.hdop.hdop(), 5, 2, buf)));
    gpxFile.print(F("</hdop>"));
  }

  gpxFile.print(F("</trkpt>\n"));

  gpxFile.print(F(GPX_EPILOGUE));

  gpxFile.flush();
}

void Logger_setup()
{
  switch(settings->logger)
  {
  case LOGGER_SD:
    //if (!SD.begin()) {
    if (!SD.begin(PIN_SD_CS, PIN_SD_MOSI, PIN_SD_MISO, PIN_SD_CLK)) {
      Serial.println("SD Init failed!");
      return;
    }
    sd_avail = 1;
    //sd = SD.open("/");
    if (sd_avail && isValidGNSSFix())
      startFilesOnSdNoSync();
    break;
  case LOGGER_OFF:
  default:
    break;
  }
}

void Logger_loop()
{
  switch(settings->logger)
  {
  case LOGGER_SD:
    if (sd_avail && isValidGNSSFix()) {
      unsigned int curTime = now();
      if (!gpxFile)
        startFilesOnSdNoSync();
      else if (curTime - prevTime >= SAMPLE_INTERVAL) {
        // TODO: Write whenever there is new data (trust GPS is set at 1Hz).
        prevTime = curTime;
        writeGpxSampleToSd();
      }
    }
    break;
  case LOGGER_OFF:
  default:
    break;
  }
}

void Logger_fini()
{
  switch(settings->logger)
  {
  case LOGGER_SD:
  SD.end();
    break;
  case LOGGER_OFF:
  default:
    break;
  }
}

#else

#define LOGFILE "/Logfile.txt"

File LogFile;
FSInfo fs_info;
FtpServer ftpSrv;   //set #define FTP_DEBUG in ESP8266FtpServer.h to see ftp verbose on serial

void Logger_setup()
{
  char LogFilename[] = LOGFILE;


  if (SPIFFS.begin()) {
    Serial.println(F("SPIFFS volume is mounted successfully."));       

      SPIFFS.info(fs_info);
      
      Serial.println();
      Serial.print(F("Total bytes: "));
      Serial.println(fs_info.totalBytes);
      Serial.print(F("Used bytes: "));
      Serial.println(fs_info.usedBytes);
      Serial.print(F("Block size: "));
      Serial.println(fs_info.blockSize);
      Serial.print(F("Page size: "));
      Serial.println(fs_info.pageSize);

    //if (SPIFFS.exists(LogFilename)) SPIFFS.remove(LogFilename); 
  
    LogFile = SPIFFS.open(LogFilename, "a+");
  
    if (!LogFile) {
      Serial.print(F("Unable to open log file: "));
      Serial.println(LogFilename);
    } else {
      LogFile.println();
      LogFile.println(F("******* Logging is restarted *******"));
      LogFile.print(F("*** Storage free space: "));
      LogFile.print(fs_info.totalBytes - fs_info.usedBytes);    
      LogFile.println(F(" bytes ***"));
  
      //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
      ftpSrv.begin("softrf","softrf");    
    };
  } else {
    Serial.println(F("ERROR: Unable to mount SPIFFS volume."));      
  };

}

void Logger_loop()
{
  ftpSrv.handleFTP();
}

void Logger_fini()
{
  LogFile.close();
  SPIFFS.end();
}

#endif /* SDGPXLOG */

#endif /* LOGGER_IS_ENABLED */
