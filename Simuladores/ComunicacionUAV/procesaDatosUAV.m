function procesaDatosUAV(Datos,nombre)
switch nargin
    case 0
        disp('Introduce conjunto de datos a procesar');
        return
    case 1
        Datos.signals(1).values(isnan(Datos.signals(1).values))=0;
        assignin('base','pIMU1',squeeze(Datos.signals(1).values(1,11,1:end-1)));
        assignin('base','qIMU1',squeeze(Datos.signals(1).values(1,12,1:end-1)));
        assignin('base','rIMU1',squeeze(Datos.signals(1).values(1,13,1:end-1)));
        assignin('base','pIMU2',squeeze(Datos.signals(1).values(1,14,1:end-1)));
        assignin('base','qIMU2',squeeze(Datos.signals(1).values(1,15,1:end-1)));
        assignin('base','rIMU2',squeeze(Datos.signals(1).values(1,16,1:end-1)));
        assignin('base','pIMU3',squeeze(Datos.signals(1).values(1,17,1:end-1)));
        assignin('base','qIMU3',squeeze(Datos.signals(1).values(1,18,1:end-1)));
        assignin('base','rIMU3',squeeze(Datos.signals(1).values(1,19,1:end-1)));
        assignin('base','axIMU1',squeeze(Datos.signals(1).values(1,23,1:end-1)));
        assignin('base','ayIMU1',squeeze(Datos.signals(1).values(1,24,1:end-1)));
        assignin('base','azIMU1',squeeze(Datos.signals(1).values(1,25,1:end-1)));
        assignin('base','axIMU2',squeeze(Datos.signals(1).values(1,26,1:end-1)));
        assignin('base','ayIMU2',squeeze(Datos.signals(1).values(1,27,1:end-1)));
        assignin('base','azIMU2',squeeze(Datos.signals(1).values(1,28,1:end-1)));
        assignin('base','axIMU3',squeeze(Datos.signals(1).values(1,29,1:end-1)));
        assignin('base','ayIMU3',squeeze(Datos.signals(1).values(1,30,1:end-1)));
        assignin('base','azIMU3',squeeze(Datos.signals(1).values(1,31,1:end-1)));
        assignin('base','tPlca',squeeze(Datos.signals(1).values(1,1,1:end-1)));
        assignin('base','t',squeeze(Datos.time(1:end-1)));

    case 2
        Datos.signals(1).values(isnan(Datos.signals(1).values))=0;
        pIMU1  = squeeze(Datos.signals(1).values(1,11,1:end-1));
        qIMU1  = squeeze(Datos.signals(1).values(1,12,1:end-1));
        rIMU1  = squeeze(Datos.signals(1).values(1,13,1:end-1));
        pIMU2  = squeeze(Datos.signals(1).values(1,14,1:end-1));
        qIMU2  = squeeze(Datos.signals(1).values(1,15,1:end-1));
        rIMU2  = squeeze(Datos.signals(1).values(1,16,1:end-1));
        pIMU3  = squeeze(Datos.signals(1).values(1,17,1:end-1));
        qIMU3  = squeeze(Datos.signals(1).values(1,18,1:end-1));
        rIMU3  = squeeze(Datos.signals(1).values(1,19,1:end-1));
        axIMU1 = squeeze(Datos.signals(1).values(1,23,1:end-1));
        ayIMU1 = squeeze(Datos.signals(1).values(1,24,1:end-1));
        azIMU1 = squeeze(Datos.signals(1).values(1,25,1:end-1));
        axIMU2 = squeeze(Datos.signals(1).values(1,26,1:end-1));
        ayIMU2 = squeeze(Datos.signals(1).values(1,27,1:end-1));
        azIMU2 = squeeze(Datos.signals(1).values(1,28,1:end-1));
        axIMU3 = squeeze(Datos.signals(1).values(1,29,1:end-1));
        ayIMU3 = squeeze(Datos.signals(1).values(1,30,1:end-1));
        azIMU3 = squeeze(Datos.signals(1).values(1,31,1:end-1));
        tPlca  = squeeze(Datos.signals(1).values(1,1,1:end-1));
        t = squeeze(Datos.time(1:end-1));
        save(nombre, "pIMU1","qIMU1","rIMU1","pIMU2","qIMU2","rIMU2","pIMU3","qIMU3","rIMU3","axIMU1","ayIMU1","azIMU1","axIMU2","ayIMU2","azIMU2","axIMU3","ayIMU3","azIMU3","tPlca","t");
end
end

    
