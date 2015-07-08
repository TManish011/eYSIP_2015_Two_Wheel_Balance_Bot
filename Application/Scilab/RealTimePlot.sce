// -----------------------------------------------------------------------------------------------------
//eYSIP-2015
//PC Controlled Two Wheel Balanced Bot
//Author List: B Suresh, Ramiz Hussain, Devendra Kr Jangid
//Mentors: Piyush Manavar, Saurav Shandilya
//-----------------------------------------------------------------------------------------------------
//IMPORTANT:Do not execute this program until the bot has begun sending the values
//CAUTION:Press the STOP button before the turning off the bot
//(***This is mainly because the slReadByte waits until atleast a single value is obtained and if the data is not sent it waits forever==>Scilab will crash***)
//-----------------------------------------------------------------------------------------------------
//INITIALISATION:
// 1.Go to the directory containing "slLoadLib.sci".
// 2.Execute the "slLoadLib.sci" file to load the library.
// 3.Enter the COMPORT number in the beginning of the following code(this can be obtained from 'Device 
//   Manager' in Windows)
// 4.Execute this gui file.
//-----------------------------------------------------------------------------------------------------
//This program creates Real-time plots of Input angle(in BLUE) and PID Output(in RED) versus Time(in tenths of seconds)
//-----------------------------------------------------------------------------------------------------
//Data Reception Format:
//To get the original values on the PC end we can do the following:
//First byte:0xFF           (marker used)
//Second byte:<received_data>-100;
//Third byte:(<received_data>/2)-127;
//-----------------------------------------------------------------------------------------------------
//This code uses the SISELI serial library Version 0.7 for serial communication

COMPORT=8;
// mount library on handle "h"
//slLoad();
h=slMount()

// handle "h": check availability of specified COM port
slCheck(h,COMPORT)

// handle "h": configure port
slConfig(h,9600,8,0,1);

// handle "h": open port
if slOpen(h,COMPORT) then
    printf("Port 8 open\n")
else
    printf("Port over board!\n")
    abort
end

n=1000;         //Time of plot in 10*seconds(eg:100seconds==>n=1000)
i=2;            //variable for time
t=1;            
prevdata=0;     //initial value of following variables is zero
prevdata2=0;
data=0;
data2=0;

//-----------------------------GUI definition and entities-----------------------------------------------

f=figure('figure_position',[600,50],'figure_size',[656,532],'auto_resize','on','background',[33],'figure_name','Graphic window number %d');
//////////
delmenu(f.figure_id,gettext('File'))
delmenu(f.figure_id,gettext('?'))
delmenu(f.figure_id,gettext('Tools'))
toolbar(f.figure_id,'off')
handles.dummy = 0;
handles.stopb=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','center','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.9,0.9,0.1,0.1],'Relief','default','SliderStep',[0.01,0.1],'String','STOP','Style','pushbutton','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','stopb','Callback','stopb_callback(handles)')

//////////
// Callbacks are defined as below. Please do not delete the comments as it will be used in coming version
//////////
//Callback function of stopbutton
function stopb_callback(handles)        
//Write your callback for  stopb  here
//slClose(h);
slUMount(h);
abort;
endfunction

plot(0,20)
a=gca();                                // Handle on axes entity
a.y_location = "origin";      
tic();
while(i<=n)
    i=2+(10*toc());
    slFlush(h);                         //flushing buffer values

    if slReadByte(h,1)==255 then        //Check for marker byte
        data=slReadByte(h,1);           //Read first byte
        data2=slReadByte(h,1);          //Read second byte
        data=data-100;                  //Reading based on data format
        data2=2*(data2-127);
    plot(i,data,'b-o');                 //Plot angle in blue
    xsegs([t,i],[prevdata,data]);       //join line
    plot(i,data2,'r-o');                //Plot PID Output in red
    xsegs([t,i],[prevdata2,data2]);
    a.x_location = "middle";            
    a.data_bounds=[i-100,-255;i+1,255]; //restricting plot for the last 10 seconds
    drawnow();                          //Real time plotting
    a.tight_limits(1);                  //To have a dynamic plot
    t=i;
    prevdata=data;                      //Store variables for subsequent usage
    prevdata2=data2;
    end
end
slClose(h);                             //Close Port
ret = slUMount(h);
   


