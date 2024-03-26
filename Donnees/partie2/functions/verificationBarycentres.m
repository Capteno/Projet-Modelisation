function verificationBarycentres(C_g,im_mask,i)

load("dino_Ps.mat");

for k = 1:size(C_g,1)
    o = P{i}*C_g(k,:)';
    o = o./repmat(o(3,:),3,1);
    imshow(im_mask(:,:,i));
    hold on;
    plot(o(2,:),o(1,:),'rx');
    title("Barycentre n°" + k + " sur l'Image : " + i);
    pause;
    close;
end
end

