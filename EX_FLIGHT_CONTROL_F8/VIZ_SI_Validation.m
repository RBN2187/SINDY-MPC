 
%% Show results for system identification [training, validation]

clear ph
darkBackground = [0.08 0.10 0.14];
lightText = [0.92 0.95 0.98];
trueColors = [0.20 0.80 1.00; 1.00 0.58 0.20; 0.45 1.00 0.58];
modelColors = [0.60 0.90 1.00; 1.00 0.78 0.45; 0.70 1.00 0.78];

figure('Color',darkBackground); box on; hold on
set(gca,'Color',darkBackground,'XColor',lightText,'YColor',lightText, ...
	'GridColor',[0.35 0.40 0.48],'LineWidth',1,'FontSize',14)
ph(1) = plot([tA(1),tA(1)],[min(xB(:)) max([xA(:);xB(:)])],':', ...
	'Color',[0.65 0.70 0.78],'LineWidth',1.5);
ylim([min(xB(:)) max([xA(:);xB(:)])])
ph(2) = plot([t;tA],[x(:,1);xA(:,1)],'Color',trueColors(1,:),'LineWidth',1.5);
ph(3) = plot([t;tA],[x(:,2);xA(:,2)],'Color',trueColors(2,:),'LineWidth',1.5);
ph(4) = plot([t;tA],[x(:,3);xA(:,3)],'Color',trueColors(3,:),'LineWidth',1.5);
ph(5) = plot(tB,xB(:,1),'-.','Color',modelColors(1,:),'LineWidth',2);
ph(6) = plot(tB,xB(:,2),'-.','Color',modelColors(2,:),'LineWidth',2);
ph(7) = plot(tB,xB(:,3),'-.','Color',modelColors(3,:),'LineWidth',2);
grid off
xlim([0 tB(end)])
xlabel('Time','Color',lightText)
ylabel('State','Color',lightText)
title('F8 SINDYc Training and Validation','Color',lightText)
lh = legend(ph,{'Training/validation boundary','True angle of attack', ...
	'True pitch angle','True pitch rate','SINDYc angle of attack', ...
	'SINDYc pitch angle','SINDYc pitch rate'},'TextColor',lightText, ...
	'Color',darkBackground,'Location','best');
set(gcf,'Position',[100 100 300 200])
set(gcf,'PaperPositionMode','auto')
print('-depsc2', '-loose','-cmyk', [figpath,'EX_',SystemModel,'_SI_',ModelName,'_',InputSignalType,'_Validation_OneFig.eps']);

%% Actuation signal
clear ph
figure('Color',darkBackground); box on; hold on
set(gca,'Color',darkBackground,'XColor',lightText,'YColor',lightText, ...
	'GridColor',[0.35 0.40 0.48],'LineWidth',1,'FontSize',14)
ph(1) = plot([tA(1),tA(1)],[-15 260],':','Color',[0.65 0.70 0.78], ...
	'LineWidth',1.5);
ph(2) = plot(t,u,'-','Color',[1.00 0.58 0.20],'LineWidth',1.5);
ph(3) = plot(tv,uv,'-','Color',[0.20 0.80 1.00],'LineWidth',1.5);
grid off
ylim([min([u uv])+0.05*min([u uv]) max([u uv])+0.05*max([u uv])])
xlim([0 tB(end)])
xlabel('Time','Color',lightText)
ylabel('Control input','Color',lightText)
title('F8 SINDYc Actuation Signal','Color',lightText)
legend(ph,{'Training/validation boundary','Training input','Validation input'}, ...
	'TextColor',lightText,'Color',darkBackground,'Location','best')
set(gcf,'Position',[100 100 300 200])
set(gcf,'PaperPositionMode','auto')
print('-depsc2', '-loose','-cmyk', [figpath,'EX_',SystemModel,'_SI_',ModelName,'_',InputSignalType,'_Actuation_OneFig.eps']);