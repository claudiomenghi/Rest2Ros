package se.got;

import java.io.IOException;
import java.io.InputStream;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JOptionPane;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.apache.http.message.BasicNameValuePair;

public class MissionSender {

	public void send(String mission, String ip, String port)  {

		System.out.println("Sending the mission "+mission+ "using a POST request");

		CloseableHttpClient httpclient = HttpClients.createDefault();
		HttpPost httppost = new HttpPost("http://"+ip+":"+port);

		List<NameValuePair> params = new ArrayList<NameValuePair>(1);
		//String sentMission=URLEncoder.encode(mission, "UTF-8");
		String sentMission=mission;
		System.out.println("mission sent: "+sentMission);
		params.add(new BasicNameValuePair("mission",sentMission ));

		try {
			httppost.setEntity(new UrlEncodedFormEntity(params, "UTF-8"));
		} catch (UnsupportedEncodingException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		
		HttpResponse response;
		try {
			response = httpclient.execute(httppost);
			HttpEntity entity = response.getEntity();

			if (entity != null) {
				InputStream instram = entity.getContent();
				instram.close();
				JOptionPane.showMessageDialog(null, "Mission correctly sent!");
				
			}
			else {
				JOptionPane.showMessageDialog(null, "Mission sent to the robot but no answer received!");
			}
		} catch (IOException e) {
			e.printStackTrace();
			JOptionPane.showMessageDialog(null, "Cannot connect with the robot!");
		}

		
	}

}
