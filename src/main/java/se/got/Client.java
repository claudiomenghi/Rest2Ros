import javax.net.ssl.HttpsURLConnection;

import java.io.BufferedReader; 
import java.io.DataOutputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.HttpURLConnection;
import java.io.OutputStream;
import java.net.URLEncoder;
import org.omg.CORBA.NameValuePair;
import java.util.List;
import java.util.ArrayList;
public class Client{

	public static void main(String[] args) throws Exception{
		
		while(true){
			System.out.println("sending post");
		
		HttpClient httpclient=HttpClients.createDefault(); 
		HttpPost httppost=new HttpPost("http://127.0.0.1:8886");

		List<NameValuePair> params=new ArrayList<NameValuePair>(1);
		params.add(new BasicNameValuePair("prop", "aaa"));
		
		httppost.setEntity(new UrlEncodedFormEntity(params, "UTF-8"));

		HttpResponse response=httpclient.execute(httppost);

		HttpEntity entity=response.getEntity();		
		
		if(entity !=null){
			InputStream instram=entity.getContent();
			instram.clos();
		}
		}
	}
}
