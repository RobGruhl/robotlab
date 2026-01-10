Additional details
Usage instructions
Prerequisites: To create and use Workstation instances you will need:
AWS Account
AWS Key Pair created for authentication
AWS security group to control access to ports
Port 22 for SSH
Port 8443 for connecting with NICE DCV
PuTTY to SSH into the AMI instance (Windows)
NICE DCV client
Launch the AMI: Navigate to the AWS Omniverse AMI marketplace product page.
To create your own AWS instance, select the View purchase options button
If you have not already subscribed to the software, you will need to Accept Terms the first time. This may take a few minutes to complete.
When the subscription is completed, click the Continue to Configuration button
On the Configure this software page list, click the Continue to Launch button
On the Launch this software page:
Set the Choose Action option to Launch through EC2
Click the Launch button
When the EC2 page opens, name your instance
Set the Instance type to g6e.2xlarge if not already listed
Set the Key Pair (login) to use your Key Pair file
In the Network settings section, select the Select existing security group option. In the Common security groups dropdown select your security group
In the Summary section on the right side of the page, click Launch instance
Find your named instance in the table. It will take a few minutes for the instance state to change from Initializing to Running
Connect to the AMI Instance Before you log in, make sure that:
Your AMI instance is running
PuTTY is installed
NICE DCV Client is installed
Key Pair created
Follow the instructions below depending on the OS you are running and the instance type.

a) Using PuTTY to connect for Linux instances - Copy the Public IP Address of your instance. You can find this by - Pick the checkbox next to your instance to select it - In the information panel below the table, find the Public IPv4 address and copy it - Open up PuTTY - In the Host Name (or IP Address) input paste your instances Public IPv4 address - Expand Connection > SSH > Auth > Credentials. Browse to the location of your Key Pair, and select it - Select Open in the PuTTY dialog to connect. - When you are connected to the AMI, change the password. The password needs to be changed in order for the NICE DCV connection to work. - Change the password for the ubuntu account in order to use the Amazon DCV client. Using sudo passwd ubuntu The password needs to be set via SSH/PuTTY each time a new instance is created, this is by design for security reasons - Enter a new password. Check your session is running by using sudo dcv list-sessions. There should be a console session running. b) Using EC2 to Connect for Windows Instances - Select your instance from the EC2 page and from the toolbar select Connect - On the Connect to instance page select the RDP Client tab - Set your username and then select Get password - Upload your private key file associated with the instance and select Decrypt password - Use this username and password to log in when you connect with DCV Client c) Connect to the Instance with DCV Client - Open the Amazon DCV Client and enter the public IP address of your instance in this format (https://<public IPv4>:8443), followed by Connect. - If you see the Server Identity Check message, select Trust and Connect - Log in using the Username: ubuntu and the password that you just set in PuTTY followed by the Login button - The Ubuntu desktop GUI will now be displayed in the Amazon DCV window
Congratulations! You have now logged in to your AMI instance. It is ready for use.