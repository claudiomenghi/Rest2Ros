package se.got;

import java.io.InputStream;
import java.net.URLEncoder;
import java.util.ArrayList;
import java.util.List;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.apache.http.message.BasicNameValuePair;

public class MissionSender {

	public void send(String mission) throws Exception {

		System.out.println("Sending the mission using a POST request");

		CloseableHttpClient httpclient = HttpClients.createDefault();
		HttpPost httppost = new HttpPost("http://127.0.0.1:13000");

		List<NameValuePair> params = new ArrayList<NameValuePair>(1);
		String sentMission=URLEncoder.encode(mission, "UTF-8");
		System.out.println("mission send: "+sentMission);
		params.add(new BasicNameValuePair("mission",sentMission ));

		httppost.setEntity(new UrlEncodedFormEntity(params, "UTF-8"));

		HttpResponse response = httpclient.execute(httppost);

		HttpEntity entity = response.getEntity();

		if (entity != null) {
			InputStream instram = entity.getContent();
			instram.close();
		}
	}

}
