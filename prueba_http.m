function prueba_http
    plot_data=[];
    val_pasado=[0,0,0,0];
    for i=0:20
        value = urlread('http://13.78.167.106/data.txt');
        val = str2num(value);
        %if val==val_pasado
         %   break;
        %end%
        val_pasado = val;
        plot_data=[plot_data; val];
        plot(plot_data);
        pause(2)
        %%linkdata on;
    end