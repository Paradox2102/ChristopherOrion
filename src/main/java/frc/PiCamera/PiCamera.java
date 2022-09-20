/*
 *	  Copyright (C) 2020  John H. Gaby
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, version 3 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *    
 *    Contact: robotics@gabysoft.com
 */

package frc.PiCamera;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

/**
 * 
 * @author John Gaby
 * 
 * @brief The PiCamera class handles communication with the Raspberry Pi Camera.
 * 
 *        This class allows you to connect to the Data server of the Raspberry
 *        Pi and receive information about the image frames processed by the
 *        camera.
 *
 *        It also implements a time sync protocol to synchronize the clocks and
 *        performs keep-alive functions that detect a disconnect.
 *
 */
public class PiCamera implements Network.NetworkReceiver {
	/**
	 * @brief The PiCameraStats class collects the current camera performance
	 *        statistics.
	 *
	 */
	public class PiCameraStats {
		public int m_averageDelay = 0; // !<Specifies the average frame delay
		public int m_maxDelay = 0; // !<Specifies the max frame delay
		public int m_minDelay = Integer.MAX_VALUE; // !<Specifies the min frame delay
		public int m_lostFrames; // !<Specifies the number of lost frames
		public long m_time; // !<Specifies the elapsed time

		public PiCameraStats(int avgDelay, int maxDelay, int minDelay, int lostFrames, long time) {
			m_averageDelay = avgDelay;
			m_maxDelay = maxDelay;
			m_minDelay = minDelay;
			m_lostFrames = lostFrames;
			m_time = time;
		}
	}

	/**
	 * @brief The PiCamerPoint specifies an (x,y) position
	 *
	 */
	public class PiCameraPoint {
		public int m_x; // !<Specifies the horizontal position in pixels from the left
		public int m_y; // !<Specifies the vertical position in pixels from the top

		public PiCameraPoint(int x, int y) {
			m_x = x;
			m_y = y;
		}
	}

	/**
	 * @brief The PiCameraRect specifies an rectangular region
	 *
	 */
	public class PiCameraRect {
		public int m_left; // !<Specifies the left edge of the region in pixels
		public int m_top; // !<Specifies the top edge of the region in pixels
		public int m_right; // !<Specifies the right edge of the region in pixels
		public int m_bottom; // !<Specifies the bottom edge of the region in pixels

		public PiCameraRect(int left, int top, int right, int bottom) {
			m_left = left;
			m_top = top;
			m_right = right;
			m_bottom = bottom;
		}
	}

	/**
	 * @brief The PiCameraRegion specifies a single detected region
	 *
	 */
	public class PiCameraRegion {
		public int m_color; // !<Specifies the region color [0..3]
		public int m_topLeft; // !<Not currently used
		public int m_topRight; // !<Not currently used
		public PiCameraRect m_bounds; // !<Specifies the bounding rectangle for the region

		// ! @cond PRIVATE
		public PiCameraRegion(int color, int left, int top, int right, int bottom, int topLeft, int topRight) {
			m_color = color;
			m_topLeft = topLeft;
			m_topRight = topRight;
			m_bounds = new PiCameraRect(left, top, right, bottom);
		}
		// ! @endcond

		/*
		 * public PiCameraPoint GetTopCenter() { return(new
		 * PiCameraPoint((m_bounds.m_left + m_bounds.m_right)/2, (m_topLeft +
		 * m_topRight)/2)); }
		 */
	}

	/**
	 * @brief The PiCameraRegions specifies list of detected regions
	 *
	 *        The list will contain up to the max regions specified using the
	 *        ImageViewer and will be sorted from the largest to smallest area.
	 *
	 */
	public class PiCameraRegions {
		public int m_targetVertPos; // !<Specifies the vertical target position as set by the ImageViewer
		public int m_targetHorzPos; // !<Specifies the horizontal target position as set by the ImageViewer
		public int m_frameNo; // !<Specifies the frame #
		public int m_width; // !<Specifies the width of the camera image (e.g. 640)
		public int m_height; // !<Specifies the height of the camera image (e.g. 480)
		public int m_lostFrames; // !<Specifies the number of lost frames
		public int m_profile; // !<Specifies which capture profile is in effect
		public long m_captureTime; // !<Specifies the time at which the image was acquired in ms
		public int m_procTime; // !<Specifies the time required to process this image in ms
		public ArrayList<PiCameraRegion> m_regions = new ArrayList<PiCameraRegion>();

		// ! @cond PRIVATE
		public PiCameraRegions(int frameNo, int targetVertPos, int targetHorzPos, int width, int height, int lostFrames,
				long captureTime, int procTime, int profile) {
			Logger.Log("PiCamera", -1, String.format("PiCameraRegions(): width = %d, height = %d", width, height));

			m_frameNo = frameNo;
			m_targetVertPos = targetVertPos;
			m_targetHorzPos = targetHorzPos;
			m_width = width;
			m_height = height;
			m_lostFrames = lostFrames;
			m_profile = profile;
			m_captureTime = captureTime;
			m_procTime = procTime;
		}

		public void AddRegion(int color, int left, int top, int right, int bottom, int topLeft, int topRight) {
			m_regions.add(new PiCameraRegion(color, left, top, right, bottom, topLeft, topRight));
		}
		// ! @endcond

		/**
		 * Returns a count of the regions
		 * 
		 */
		public int GetRegionCount() {
			return (m_regions.size());
		}

		/**
		 * Returns a specified region
		 * 
		 * @param region - Specifies the region to return
		 */
		public PiCameraRegion GetRegion(int region) {

			if ((region >= 0) && (region < m_regions.size())) {

				return (m_regions.get(region));
			}
			// System.out.println(region);
			// Logger.Log("PiCamera", 2, "" + region);
			return (null);
		}
	}

	private static final int k_syncRetry = 5000;
	private static final int k_syncFirst = 1000;

	private Network m_network = null;
	private PiCameraRegions m_regions = null;
	private PiCameraRegions m_nextRegions = null;
	private long m_syncTime;
	private int m_averageDelayCount = 0;
	private int m_averageDelayMax = 30;
	private int m_averageDelaySum = 0;
	private int m_averageDelay = 0;
	private int m_maxDelay = 0;
	private int m_minDelay = Integer.MAX_VALUE;
	private int m_lostFrames;
	private int m_lastLostFrame = 0;
	private long m_startTime = 0;
	private boolean m_connected = false;
	private Timer m_watchdogTimer = new Timer();
	private long m_lastMessage;
	private static final int k_timeout = 5000;

	public PiCamera() {
		m_watchdogTimer.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
				if (m_connected) {
					Logger.Log("PiCamera", -1, "WatchDog");

					m_network.SendMessage("k");

					if (m_lastMessage + k_timeout < System.currentTimeMillis()) {
						Logger.Log("PiCamera", 3, "Network timeout");
						m_network.CloseConnection();
					}
				}
			}
		}, 1000, 1000);
	}

	/**
	 * Returns true if connected to the camera
	 * 
	 */
	public boolean IsConnected() {
		return (m_connected);
	}

	// ! @cond PRIVATE
	public static int[] ParseIntegers(String str, int count) {
		int[] args = new int[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Integer.parseInt(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	public static long[] ParseLong(String str, int count) {
		long[] args = new long[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Long.parseLong(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	long m_pingTime = 0;

	public void Ping() {
		m_pingTime = GetTimeMs();

		m_network.SendMessage("p");
	}
	// ! @endcond

	/**
	 * Instructs the Raspberry Pi to dump the last N frame to disk. This can be
	 * useful in debugging to obtain a record of what the camera saw.
	 * 
	 * @param count - Count of frames to dump (max 120)
	 * 
	 */
	public void DumpFrames(int count) {
		Logger.Log("PiCamera", 1, String.format("DumpFrames(%d)", count));
		m_network.SendMessage(String.format("d %d", count));
	}

	PrintWriter m_log = null;
	long m_logTime = 0;

	/**
	 * Starts logging the data received from the Raspberry Pi locally
	 * 
	 * @param path - Specifies the path for the log file.
	 */
	public void StartLogging(String path) {
		synchronized (this) {
			if (m_log != null) {
				m_log.close();
				m_log = null;
			}

			try {
				m_log = new PrintWriter(path);
				m_logTime = System.currentTimeMillis();
				m_lastLostFrame = m_lostFrames;
				m_minDelay = Integer.MAX_VALUE;
				m_maxDelay = 0;

				m_log.println("time,frame,#Regions,lost,cap delay,proc delay,left,top,right,bottom");
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Ends logging and closes the log file
	 * 
	 */
	public void EndLogging() {
		synchronized (this) {
			if (m_log != null) {
				m_log.close();
				m_log = null;
			}
		}
	}

	/**
	 * The Raspberry Pi controls pins 11 and 12 and these pins can be used to
	 * control relays which can control lights for the camera
	 * 
	 * @param value - Bit zero controls pin 11 and bit one controls pin 12
	 */
	public void SetLight(int value) {
		m_network.SendMessage(String.format("L %d", value));
	}
	
	public void SendJpeg(boolean send)
	{
		m_network.SendMessage("J" + (send ? "y" : "n"));
	}

	/**
	 * Specifies which capture profile should be used.
	 * 
	 * @param profile - Specifies the profile to use (0-3)
	 */
	public void SetProfile(int profile) {
		if ((profile >= 0) && (profile < 4)) {
			m_network.SendMessage(String.format("P %d", profile));
		}
	}

	/**
	 * Instructs the Raspberry Pi to start logging the camera data on the Pi's disk
	 * This can be useful for debugging
	 *
	 */
	public void StartPiLog() {
		m_network.SendMessage("ls");
	}

	/**
	 * Ends the Raspberry Pi logging
	 *
	 */
	public void EndPiLog() {
		m_network.SendMessage("le");
	}

	/**
	 * Connects to the Raspberry Pi Data server
	 *
	 * @param host - Specifies IP for the host
	 * @param port - Specifies the port (default is 5800)
	 */
	public void Connect(String host, int port) {
		m_network = new Network();

		m_startTime = System.currentTimeMillis();

		m_network.Connect(this, host, port);

	}

	private void TimeSync() {
		long time = GetTimeMs();
		if (time > m_syncTime) {
			Logger.Log("PiCamera", -1, "TimeSync()");

			m_network.SendMessage(String.format("T1 %d", GetTimeMs()));

			m_syncTime = time + k_syncRetry;
		}

	}

	private void ProcessCameraFrame(String args) {
		long a[] = ParseLong(args, 9);

		if (a != null) {
			m_nextRegions = new PiCameraRegions((int) a[0], (int) a[1], (int) a[2], (int) a[3], (int) a[4], (int) a[5],
					a[6], (int) a[7], (int) a[8]);

			int delay = (int) (System.currentTimeMillis() - m_nextRegions.m_captureTime);
			int averageDelay = -1;

			m_averageDelaySum += delay;
			if (++m_averageDelayCount >= m_averageDelayMax) {
				averageDelay = m_averageDelaySum / m_averageDelayCount;

				m_averageDelayCount = 0;
				m_averageDelaySum = 0;
			}

			synchronized (this) {
				if (averageDelay > 0) {
					m_averageDelay = averageDelay;
				}

				if (delay > m_maxDelay) {
					m_maxDelay = delay;
				}
				if (delay < m_minDelay) {
					m_minDelay = delay;
				}

				m_lostFrames = (int) a[5];
			}
		}
	}

	/**
	 * Returns the current camera performance stats
	 *
	 */
	public PiCameraStats GetStats() {
		synchronized (this) {
			return (new PiCameraStats(m_averageDelay, m_maxDelay, m_minDelay, m_lostFrames - m_lastLostFrame,
					System.currentTimeMillis() - m_startTime));
		}
	}

	/**
	 * Clears the camera performance stats
	 *
	 */
	public void ClearStats() {
		synchronized (this) {
			m_maxDelay = 0;
			m_minDelay = Integer.MAX_VALUE;
			m_lostFrames = 0;
			m_startTime = System.currentTimeMillis();
		}
	}

	private void ProcessCameraRegion(String args) {
		int a[] = ParseIntegers(args, 7);

		if ((a != null) && (m_nextRegions != null)) {
			m_nextRegions.AddRegion(a[0], a[1], a[2], a[3], a[4], a[5], a[6]);
		}
	}

	private void LogFrame() {
		long curTime = System.currentTimeMillis();
		int size = m_regions.m_regions.size();

		m_log.print(String.format("%d,%d,%d,%d,%d,%d", curTime - m_logTime, m_regions.m_frameNo, size,
				m_regions.m_lostFrames, curTime - m_regions.m_captureTime, m_regions.m_procTime));

		if (size > 0) {
			PiCameraRegion region = m_regions.m_regions.get(0);

			m_log.print(String.format(",%d,%d,%d,%d", region.m_bounds.m_left, region.m_bounds.m_top,
					region.m_bounds.m_right, region.m_bounds.m_bottom));
		}

		m_log.println("");

		if ((m_regions.m_frameNo % 30) == 0) {
			m_log.flush(); // flush every 30 frames
		}
	}

	private void ProcessCameraEnd(String args) {
		synchronized (this) {
			m_regions = m_nextRegions;
			m_nextRegions = null;

			if (m_log != null) {
				LogFrame();
			}
		}

		TimeSync();
	}

	/**
	 * Returns the latest set of camera regions. Note that the data is received from
	 * the camera in a separate thread so calling this twice in a row can generate a
	 * different result. Once you retrieve and instance of PiCameraRegions however,
	 * you can be assured that it will NOT be modified when a new frame is received.
	 *
	 */
	public PiCameraRegions GetRegions() {
		synchronized (this) {
			return (m_regions);
		}

	}

	private long GetTimeMs() {
		return System.currentTimeMillis();
	}

	private void ProcessTimeSync() {
		Logger.Log("PiCamera", -1, "ProcessTimeSync()");

		long time = GetTimeMs();

		m_network.SendMessage(String.format("T2 %d", time));
	}

	// ! @cond PRIVATE
	@Override
	public void ProcessData(String data) {
		Logger.Log("PiCamera", -1, String.format("Data: %s", data));

		m_lastMessage = System.currentTimeMillis();

		switch (data.charAt(0)) {
		case 'F':
			ProcessCameraFrame(data.substring(1).trim());
			break;

		case 'R':
			ProcessCameraRegion(data.substring(1).trim());
			break;

		case 'E':
			ProcessCameraEnd(data.substring(1).trim());
			break;

		case 'p':
			Logger.Log("PiCamera", 3, String.format("Ping = %d", GetTimeMs() - m_pingTime));
			break;

		case 'T': // sync
			ProcessTimeSync();
			break;

		default:
			Logger.Log("PiCamera", 3, String.format("Invalid command: %s", data));
			break;
		}
	}

	@Override
	public void Disconnected() {
		m_connected = false;
	}

	@Override
	public void Connected() {
		m_connected = true;

		m_syncTime = GetTimeMs() + k_syncFirst;
		m_lastMessage = m_syncTime;
	}
	// ! @endcond
}
