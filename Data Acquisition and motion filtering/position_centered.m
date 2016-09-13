function c = position_centered(P_ball)
    c = [322.35 179.1];
    
    P_ballc = P_ball(1:400,:);    
    
    comparator_before = inf;
    x_min = 0;
    
    
    for cx = (c(1)-2):0.01:(c(1)+2)        
        P_c(:,1) = P_ballc(:,1)-cx;
        P_c(:,2) = P_ballc(:,2)-c(2);
        r = sqrt(P_c(:,1).^2 + P_c(:,2).^2);        
        comparator_now = std(r);
        
        if(comparator_before > comparator_now)
            comparator_before = comparator_now;
            x_min = cx;
        end
%         plot(r);
%         pause(0.01);
    end
    
    comparator_before = inf;
    y_min = 0;
    
    for cy = (c(2)-2):0.01:(c(2)+2)        
        P_c(:,1) = P_ballc(:,1)-c(1);
        P_c(:,2) = P_ballc(:,2)-cy;
        r = sqrt(P_c(:,1).^2 + P_c(:,2).^2);        
        comparator_now = std(r);
        
        if(comparator_before > comparator_now)
            comparator_before = comparator_now;
            y_min = cy;
        end
%         plot(r);
%         pause(0.01);
    end
    
    c = [x_min,y_min];

end