COMPORT=8;
// mount library on handle "1"
//slLoad();
h=slMount()

// handle "1": check availability of specified COM port
slCheck(h,COMPORT)

// handle "1": configure port
slConfig(h,9600,8,0,1);

// handle "1": open port
if slOpen(h,COMPORT) then
    printf("Port 8 open\n")
else
    printf("Port over board!\n")
    abort
end

n=1000;
i=2;
t=1;
prevdata=0;
prevdata2=0;
data=0;
data2=0;

//h=openserial(8,"9600,n,8,1",blocking=%t);
//data=zeros(n*10);

f=figure('figure_position',[300,0],'figure_size',[956,752],'auto_resize','on','background',[33],'figure_name','Balance Bot eSIP 2015');

//////////
delmenu(f.figure_id,gettext('File'))
delmenu(f.figure_id,gettext('?'))
delmenu(f.figure_id,gettext('Tools'))
toolbar(f.figure_id,'off')
handles.dummy = 0;
handles.stopb=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','center','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.9,0,0.08,0.1],'Relief','default','SliderStep',[0.01,0.1],'String','STOP','Style','pushbutton','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','stopb','Callback','stopb_callback(handles)')

//////////
// Callbacks are defined as below. Please do not delete the comments as it will be used in coming version
//////////

function stopb_callback(handles)
//Write your callback for  stopb  here
//slClose(h);
slUMount(h);
abort;
endfunction

handles.inc_lil=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','center','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.5,0.3,0.08,0.1],'Relief','default','SliderStep',[0.01,0.1],'String','+0.1','Style','pushbutton','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','inclil','Callback','inclil_callback(handles)')
handles.dec_lil=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','center','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.4,0.3,0.08,0.1],'Relief','default','SliderStep',[0.01,0.1],'String','-0.1','Style','pushbutton','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','declil','Callback','declil_callback(handles)')
handles.inc_more=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','center','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.5,0.1,0.08,0.1],'Relief','default','SliderStep',[0.01,0.1],'String','+5','Style','pushbutton','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','incmore','Callback','incmore_callback(handles)')
handles.dec_more=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','center','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.4,0.1,0.08,0.1],'Relief','default','SliderStep',[0.01,0.1],'String','-5','Style','pushbutton','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','decmore','Callback','decmore_callback(handles)')
handles.inc=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','center','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.5,0.2,0.08,0.1],'Relief','default','SliderStep',[0.01,0.1],'String','+1','Style','pushbutton','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','inc','Callback','inc_callback(handles)')
handles.dec=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','center','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.4,0.2,0.08,0.1],'Relief','default','SliderStep',[0.01,0.1],'String','-1','Style','pushbutton','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','dec','Callback','dec_callback(handles)')
handles.list=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[12],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','left','ListboxTop',[1],'Max',[1],'Min',[0],'Position',[0.25,0.17,0.0984375,0.20],'Relief','default','SliderStep',[0.01,0.1],'String','Kp|Ki|Kd|F/B','Style','listbox','Value',[3],'VerticalAlignment','middle','Visible','on','Tag','list','Callback','list_callback(handles)')
handles.tuning=uicontrol(f,'unit','normalized','BackgroundColor',[-1,-1,-1],'Enable','on','FontAngle','normal','FontName','Tahoma','FontSize',[20],'FontUnits','points','FontWeight','normal','ForegroundColor',[-1,-1,-1],'HorizontalAlignment','left','ListboxTop',[],'Max',[1],'Min',[0],'Position',[0.3340611,0,0.1831659,0.06096],'Relief','default','SliderStep',[0.01,0.1],'String','Tuning Parameters','Style','text','Value',[0],'VerticalAlignment','middle','Visible','on','Tag','tuning','Callback','')


//////////
// Callbacks are defined as below. Please do not delete the comments as it will be used in coming version
//////////



function inclil_callback(handles)
//Write your callback for  inc_lil  here
slSendByte(h,ascii("3"));
endfunction


function declil_callback(handles)
//Write your callback for  dec_lil  here
slSendByte(h,ascii("6"));
endfunction


function incmore_callback(handles)
//Write your callback for  inc_more  here
slSendByte(h,ascii("2"));
endfunction


function decmore_callback(handles)
//Write your callback for  dec_more  here
slSendByte(h,ascii("5"));
endfunction


function inc_callback(handles)
//Write your callback for  inc  here
slSendByte(h,ascii("1"));
endfunction


function dec_callback(handles)
//Write your callback for  dec  here
slSendByte(h,ascii("4"))
//disp(4);
endfunction


function list_callback(handles)
//Write your callback for  list  here
//data=ascii(get(h,'value'));
data=get(handles.list,'value');
if data ==4 then
    slSendByte(h,48);
    slSendByte(h,48);
    slSendByte(h,48);
else
    slSendByte(h,data+54);
    slSendByte(h,data+54);
    slSendByte(h,data+54);
end

//disp(data);
endfunction

subplot(211)
plot(0,20)
a=gca(); // Handle on axes entity
//a.x_location = "origin"; 
a.y_location = "origin";
//f.figure_size= [100,530];
ylabel("Input/Output value", "fontsize",2 );
xlabel("Time(in sec)","fontsize",3);
title(a,'Pid Input and Output','fontsize',4);
tic();
while(i<=n)
    //h=openserial(8,"9600,n,8,1",blocking=%t);
    i=toc();
    //TCL_EvalStr("flush.h");
    //fort("flush",h,1,"i","out",[1,1],2,"i")
    //TCL_EvalStr("flush "+h)
    //[q,flags]=serialstatus(h)
    //disp(q)
    //writeserial(h,'t');
    //data(i)=ascii(readserial(h,1));
    slFlush(h);

    if slReadByte(h,1)==255 then     
        data=slReadByte(h,1);
        //data=slReadArray(h);
        data2=slReadByte(h,1);
        data=data-100;
        data2=2*(data2-127);
    //data=data-100;
    plot(i*0.1,data,'b-o');
    xsegs([t*0.1,i*0.1],[prevdata,data]);
    plot(i*0.1,data2,'r-o');
    xsegs([t*0.1,i*0.1],[prevdata2,data2]);
    //set(a,'tight_limits',"on");
    a.x_location = "middle";
    a.data_bounds=[i-10,-255;i+1,255];
    drawnow();
    a.tight_limits(1);
    t=i;
    prevdata=data;
    prevdata2=data2;
    //disp(data);
    //disp(data2);
    //disp(--------------);
    end
    //key=input();
    //if key ~= ' ' then
    //    writeserial(h,key);
    //end
    //closeserial(h);
    //sleep(10); // wait a little to give data a chance to arrive
    //disp("----------------------");
end
//closeserial(h);
slClose(h); 
slUMount(h);
   


